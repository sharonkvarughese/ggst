#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

class TrajectoryPlotterEKF:
    def __init__(self):
        rospy.init_node('trajectory_plotter')
        
        # Get parameters
        self.scenario = rospy.get_param('~scenario', 'three_times')
        self.turn_direction = rospy.get_param('~turn_direction', 'port')
        self.boat_length = 4.06
        
        # Calculate scenario parameters
        if self.scenario == 'three_times':
            self.straight_line_length = self.boat_length * 3
            self.tactical_diameter = self.boat_length * 3
        else:
            self.straight_line_length = self.boat_length * 2
            self.tactical_diameter = self.boat_length * 2
            
        self.turn_radius = self.tactical_diameter / 2
        
        # State variables
        self.start_pos = None
        self.start_yaw = None
        self.actual_x = []
        self.actual_y = []
        self.ideal_x = []
        self.ideal_y = []
        self.errors = []
        self.times = []
        
        # Setup plots
        plt.ion()
        self.fig, (self.ax_traj, self.ax_error) = plt.subplots(1, 2, figsize=(15, 6))
        self.setup_plots()
        
        # Subscribe to EKF output
        self.ekf_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', 
                                      PoseWithCovarianceStamped, 
                                      self.ekf_callback)
        
        self.start_time = rospy.Time.now()
        self.rate = rospy.Rate(10)

    def setup_plots(self):
        self.ax_traj.set_title('Trajectory Comparison')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True)
        self.ax_traj.set_aspect('equal')
        self.ax_traj.set_xlim([-20, 20])
        self.ax_traj.set_ylim([-20, 20])
        
        # Initialize plot lines
        self.actual_line, = self.ax_traj.plot([], [], 'b-', label='Actual Path')
        self.ideal_line, = self.ax_traj.plot([], [], 'r--', label='Ideal Path')
        self.ax_traj.legend()
        
        # Error plot
        self.ax_error.set_title('Path Error')
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (m)')
        self.ax_error.grid(True)
        self.error_line, = self.ax_error.plot([], [], 'r-')

    def transform_coordinates(self, x, y):
        """Transform coordinates to align with reference frame"""
        if self.start_pos is None:
            return x, y
            
        dx = x - self.start_pos[0]
        dy = y - self.start_pos[1]
        
        if self.start_yaw is not None:
            cos_yaw = math.cos(-self.start_yaw)
            sin_yaw = math.sin(-self.start_yaw)
            x_trans = dx * cos_yaw - dy * sin_yaw
            y_trans = dx * sin_yaw + dy * cos_yaw
            return x_trans, y_trans
        
        return dx, dy

    def generate_ideal_path(self):
        """Generate ideal path in the transformed coordinate frame"""
        x_points = []
        y_points = []
        
        # Generate straight line
        straight_points = 50
        for i in range(straight_points):
            t = i / (straight_points - 1)
            x_points.append(t * self.straight_line_length)
            y_points.append(0)
        
        # Generate circle
        circle_points = 100
        center_x = self.straight_line_length
        center_y = self.turn_radius if self.turn_direction == 'port' else -self.turn_radius
        
        if self.turn_direction == 'port':
            angles = np.linspace(-np.pi/2, 3*np.pi/2, circle_points)
        else:
            angles = np.linspace(np.pi/2, -3*np.pi/2, circle_points)
            
        for angle in angles:
            x = center_x + self.turn_radius * np.cos(angle)
            y = center_y + self.turn_radius * np.sin(angle)
            x_points.append(x)
            y_points.append(y)
        
        return x_points, y_points

    def calculate_error(self, x, y):
        if not self.ideal_x:
            return 0.0
            
        min_dist = float('inf')
        for i in range(len(self.ideal_x)):
            dx = x - self.ideal_x[i]
            dy = y - self.ideal_y[i]
            dist = math.sqrt(dx*dx + dy*dy)
            min_dist = min(min_dist, dist)
        return min_dist

    def ekf_callback(self, msg):
        if self.start_pos is None:
            self.start_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            _, _, self.start_yaw = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            self.ideal_x, self.ideal_y = self.generate_ideal_path()
        
        # Transform current position
        x, y = self.transform_coordinates(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Store transformed coordinates
        self.actual_x.append(x)
        self.actual_y.append(y)
        error = self.calculate_error(x, y)
        self.errors.append(error)
        t = (rospy.Time.now() - self.start_time).to_sec()
        self.times.append(t)

    def update_plots(self):
        if not self.start_pos:
            return

        # Update actual path
        self.actual_line.set_data(self.actual_x, self.actual_y)
        
        # Update ideal path
        if self.ideal_x:
            self.ideal_line.set_data(self.ideal_x, self.ideal_y)
        
        # Update error plot
        if len(self.times) > 0:
            self.error_line.set_data(self.times, self.errors)
            self.ax_error.relim()
            self.ax_error.autoscale_view()
        
        plt.draw()
        plt.pause(0.01)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_plots()
                self.rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in plotting loop: {str(e)}")

    def shutdown_hook(self):
        try:
            plt.savefig(f'turning_circle_{self.scenario}_{self.turn_direction}_ekf.png')
            plt.close('all')
        except Exception as e:
            rospy.logerr(f"Error saving plots: {str(e)}")

if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotterEKF()
        rospy.on_shutdown(plotter.shutdown_hook)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
