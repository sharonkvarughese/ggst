#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from collections import deque

class DynamicsPlotterEKF:
    def __init__(self):
        rospy.init_node('dynamics_plotter')
        
        # Initialize buffer size
        self.buffer_size = 5000
        
        # Initialize velocity data
        self.velocity_data = {
            'time': deque(maxlen=self.buffer_size),
            'x': deque(maxlen=self.buffer_size),
            'y': deque(maxlen=self.buffer_size),
            'mag': deque(maxlen=self.buffer_size)
        }
        
        # Initialize acceleration data
        self.accel_data = {
            'time': deque(maxlen=self.buffer_size),
            'x': deque(maxlen=self.buffer_size),
            'y': deque(maxlen=self.buffer_size),
            'mag': deque(maxlen=self.buffer_size)
        }
        
        # EKF data
        self.ekf_data = {
            'time': deque(maxlen=self.buffer_size),
            'vel_x': deque(maxlen=self.buffer_size),
            'vel_y': deque(maxlen=self.buffer_size),
            'acc_x': deque(maxlen=self.buffer_size),
            'acc_y': deque(maxlen=self.buffer_size)
        }
        
        
        self.alpha_vel = 0.1  
        self.alpha_acc = 0.03 
        self.window_size = 5  
        self.vel_buffer_x = deque(maxlen=self.window_size)
        self.vel_buffer_y = deque(maxlen=self.window_size)
        self.acc_buffer_x = deque(maxlen=self.window_size)
        self.acc_buffer_y = deque(maxlen=self.window_size)
        self.last_accel = {'x': 0.0, 'y': 0.0}
        self.last_vel = {'x': 0.0, 'y': 0.0}
        
        # Setup plot window
        plt.ion()
        self.fig = plt.figure(figsize=(15, 5))
        self.setup_plots()
        
        # Setup subscribers
        self.ekf_sub = rospy.Subscriber('/robot_pose_ekf/odom_combined', 
                                      PoseWithCovarianceStamped, 
                                      self.ekf_callback)
        self.odom_sub = rospy.Subscriber('/wamv/odom', 
                                       Odometry, 
                                       self.odom_callback)
        self.imu_sub = rospy.Subscriber('/wamv/imu', 
                                      Imu, 
                                      self.imu_callback)
        
        self.start_time = rospy.Time.now()
        self.last_ekf_time = None
        self.rate = rospy.Rate(50)
        
        rospy.loginfo("EKF Dynamics plotter initialized")

    def setup_plots(self):
        # Velocity plot
        self.ax_vel = self.fig.add_subplot(121)
        self.ax_vel.set_title('Translational Velocity')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Velocity (m/s)')
        self.ax_vel.grid(True)
        self.ax_vel.set_ylim([-5, 5])
        self.ax_vel.set_xlim([-1, 10])
        
        # Acceleration plot
        self.ax_accel = self.fig.add_subplot(122)
        self.ax_accel.set_title('Translational Acceleration')
        self.ax_accel.set_xlabel('Time (s)')
        self.ax_accel.set_ylabel('Acceleration (m/s²)')
        self.ax_accel.grid(True)
        self.ax_accel.set_ylim([-4.0, 4.5])
        self.ax_accel.set_xlim([-1, 10])
        
        # Initialize plot lines
        self.vel_x_line, = self.ax_vel.plot([], [], 'b-', label='Vx')
        self.vel_y_line, = self.ax_vel.plot([], [], 'r-', label='Vy')
        self.vel_mag_line, = self.ax_vel.plot([], [], 'g-', label='V')
        self.ax_vel.legend()
        
        self.accel_x_line, = self.ax_accel.plot([], [], 'b-', label='Surge (X)')
        self.accel_y_line, = self.ax_accel.plot([], [], 'r-', label='Sway (Y)')
        self.accel_mag_line, = self.ax_accel.plot([], [], 'g-', label='Total')
        self.ax_accel.legend()
        
        plt.tight_layout()

    def filter_acceleration(self, ax_raw, ay_raw):
        peak_threshold = 1.8  # m/s²
        
        # Outlier rejection
        ax_raw = np.clip(ax_raw, -peak_threshold, peak_threshold)
        ay_raw = np.clip(ay_raw, -peak_threshold, peak_threshold)
        
        # Moving average buffer update
        self.acc_buffer_x.append(ax_raw)
        self.acc_buffer_y.append(ay_raw)
        
        # Compute moving average
        ax_ma = np.mean(self.acc_buffer_x)
        ay_ma = np.mean(self.acc_buffer_y)
        
        # Apply exponential filter
        ax_filtered = self.alpha_acc * ax_ma + (1 - self.alpha_acc) * self.last_accel['x']
        ay_filtered = self.alpha_acc * ay_ma + (1 - self.alpha_acc) * self.last_accel['y']
        
        # Store filtered values
        self.last_accel['x'] = ax_filtered
        self.last_accel['y'] = ay_filtered
        
        return ax_filtered, ay_filtered

    def filter_velocity(self, vx_raw, vy_raw):
        """Velocity filtering with moving average and exponential filter"""
        
        self.vel_buffer_x.append(vx_raw)
        self.vel_buffer_y.append(vy_raw)
 
        vx_ma = np.mean(self.vel_buffer_x)
        vy_ma = np.mean(self.vel_buffer_y)
        
        vx_filtered = self.alpha_vel * vx_ma + (1 - self.alpha_vel) * self.last_vel['x']
        vy_filtered = self.alpha_vel * vy_ma + (1 - self.alpha_vel) * self.last_vel['y']
        
        # Store filtered values
        self.last_vel['x'] = vx_filtered
        self.last_vel['y'] = vy_filtered
        
        return vx_filtered, vy_filtered
        
    def ekf_callback(self, msg):
        t = (msg.header.stamp - self.start_time).to_sec()
        
        # Extract pose and covariance
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate velocities using EKF data
        if self.last_ekf_time is not None:
            dt = (msg.header.stamp - self.last_ekf_time).to_sec()
            if dt > 0:
                if hasattr(self, 'last_ekf_pose'):
                    vx_raw = (x - self.last_ekf_pose[0]) / dt
                    vy_raw = (y - self.last_ekf_pose[1]) / dt
                    vx, vy = self.filter_velocity(vx_raw, vy_raw)
                    
                    # Calculate accelerations
                    if hasattr(self, 'last_ekf_vel'):
                        ax_raw = (vx - self.last_ekf_vel[0]) / dt
                        ay_raw = (vy - self.last_ekf_vel[1]) / dt
                        
                        # Apply acceleration filtering
                        ax_filtered, ay_filtered = self.filter_acceleration(ax_raw, ay_raw)
                        
                        # Store EKF-derived accelerations
                        self.ekf_data['time'].append(t)
                        self.ekf_data['acc_x'].append(ax_filtered)
                        self.ekf_data['acc_y'].append(ay_filtered)
                    
                    self.last_ekf_vel = [vx, vy]
                    
                    # Store EKF-derived velocities
                    self.ekf_data['vel_x'].append(vx)
                    self.ekf_data['vel_y'].append(vy)
        
        self.last_ekf_time = msg.header.stamp
        self.last_ekf_pose = [x, y]

    def odom_callback(self, msg):
        
        t = (rospy.Time.now() - self.start_time).to_sec()
        vx_raw = msg.twist.twist.linear.x
        vy_raw = msg.twist.twist.linear.y
        vx, vy = self.filter_velocity(vx_raw, vy_raw)
        v_mag = math.sqrt(vx**2 + vy**2)
        
        # Store filtered values
        self.velocity_data['time'].append(t)
        self.velocity_data['x'].append(vx)
        self.velocity_data['y'].append(vy)
        self.velocity_data['mag'].append(v_mag)

    def imu_callback(self, msg):
        
        t = (rospy.Time.now() - self.start_time).to_sec()
        
        # Get orientation
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Get raw accelerations
        ax_raw = msg.linear_acceleration.x
        ay_raw = msg.linear_acceleration.y
        
        # Remove gravity component
        g = 9.81
        ax = ax_raw + g * math.sin(pitch)
        ay = ay_raw - g * math.sin(roll) * math.cos(pitch)
        
        # Apply filtering
        ax_filtered, ay_filtered = self.filter_acceleration(ax, ay)
        
        # Calculate magnitude after filtering
        a_mag = math.sqrt(ax_filtered**2 + ay_filtered**2)
        
        # Store filtered values
        self.accel_data['time'].append(t)
        self.accel_data['x'].append(ax_filtered)
        self.accel_data['y'].append(ay_filtered)
        self.accel_data['mag'].append(a_mag)

    def update_plots(self):
        try:
            # Update velocity plot
            if len(self.velocity_data['time']) > 0:
                times = list(self.velocity_data['time'])
                self.vel_x_line.set_data(times, list(self.velocity_data['x']))
                self.vel_y_line.set_data(times, list(self.velocity_data['y']))
                self.vel_mag_line.set_data(times, list(self.velocity_data['mag']))
                
                current_max = max(times) + 2
                self.ax_vel.set_xlim(-1, current_max)
            
            # Update acceleration plot
            if len(self.accel_data['time']) > 0:
                times = list(self.accel_data['time'])
                self.accel_x_line.set_data(times, list(self.accel_data['x']))
                self.accel_y_line.set_data(times, list(self.accel_data['y']))
                self.accel_mag_line.set_data(times, list(self.accel_data['mag']))
                
                current_max = max(times) + 2
                self.ax_accel.set_xlim(-1, current_max)
            
            plt.draw()
            plt.pause(0.01)
            
        except Exception as e:
            rospy.logerr(f"Error updating plots: {str(e)}")

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_plots()
                self.rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in plotting loop: {str(e)}")

    def shutdown_hook(self):
        try:
            plt.savefig('dynamics_plots.png')
            plt.close('all')
        except Exception as e:
            rospy.logerr(f"Error saving plots: {str(e)}")

if __name__ == '__main__':
    try:
        plotter = DynamicsPlotterEKF()
        rospy.on_shutdown(plotter.shutdown_hook)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
