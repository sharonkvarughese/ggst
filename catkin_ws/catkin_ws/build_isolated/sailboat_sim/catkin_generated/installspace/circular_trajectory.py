#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TurningCircleController:
    def __init__(self):
        rospy.init_node('wamv_turning_circle_controller')
        
        # Fixed parameters
        self.boat_length = 4.06
        self.max_advance = rospy.get_param('~max_advance', 4.05)
        self.scenario = rospy.get_param('~scenario', 'two_times')
        self.straight_line = rospy.get_param('~straight_line')
        self.circle_diameter = rospy.get_param('~circle_diameter')
        self.stop_distance = rospy.get_param('~stop_distance', 0.5)
        
        # Calculated parameters
        self.turn_radius = self.circle_diameter / 2
        self.cruise_speed = 5.0
        
        # Calculate maximum turn rate for advance ≤ 4.05m
        self.max_turn_rate = self.cruise_speed / (self.max_advance / math.pi)
        self.target_turn_rate = min(self.max_turn_rate, self.cruise_speed / self.turn_radius)
        
        # Control phases
        self.phase = "STRAIGHT"  # STRAIGHT, TURN, STOP
        self.distance_traveled = 0
        self.turn_angle = 0
        
        # Initialize measurements
        self.measurements = {
            'start_pos': None,
            'turn_start_pos': None,
            'current_pos': None,
            'advance': 0,
            'transfer': 0,
            'tactical_diameter': 0,
            'turn_complete': False
        }
        
        # Setup publishers and subscribers
        self.setup_ros_interface()
        
        rospy.loginfo(f"Initialized Turning Circle Test:")
        rospy.loginfo(f"  Scenario: {self.scenario}")
        rospy.loginfo(f"  Straight Line: {self.straight_line}m")
        rospy.loginfo(f"  Circle Diameter: {self.circle_diameter}m")
        rospy.loginfo(f"  Max Advance: {self.max_advance}m")

    def setup_ros_interface(self):
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.publishers = {
            'speed_state': rospy.Publisher('speed_controller/state', Float64, queue_size=1),
            'speed_setpoint': rospy.Publisher('speed_controller/setpoint', Float64, queue_size=1),
            'turn_rate_state': rospy.Publisher('turn_rate_controller/state', Float64, queue_size=1),
            'turn_rate_setpoint': rospy.Publisher('turn_rate_controller/setpoint', Float64, queue_size=1)
        }
        
        # Subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.Subscriber('speed_controller/control_effort', Float64, self.speed_callback)
        rospy.Subscriber('turn_rate_controller/control_effort', Float64, self.turn_rate_callback)
        
        # Control inputs
        self.speed_control = 0
        self.turn_rate_control = 0

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('wamv')
            pose = msg.pose[idx]
            twist = msg.twist[idx]
            
            # Store current position
            self.measurements['current_pos'] = pose.position
            
            # Initialize start position if needed
            if not self.measurements['start_pos']:
                self.measurements['start_pos'] = pose.position
                return
            
            # Update measurements based on phase
            if self.phase == "STRAIGHT":
                self.update_straight_line_progress()
            elif self.phase == "TURN":
                self.update_turn_measurements()
            elif self.phase == "STOP":
                self.check_stop_condition()
            
            # Publish states for PID controllers
            current_speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
            self.publishers['speed_state'].publish(Float64(current_speed))
            self.publishers['speed_setpoint'].publish(Float64(self.cruise_speed))
            
            if self.phase == "TURN":
                self.publishers['turn_rate_state'].publish(Float64(twist.angular.z))
                self.publishers['turn_rate_setpoint'].publish(Float64(self.target_turn_rate))
                
        except ValueError:
            rospy.logwarn_throttle(5, "WAM-V not found in model states")

    def update_straight_line_progress(self):
        dx = self.measurements['current_pos'].x - self.measurements['start_pos'].x
        dy = self.measurements['current_pos'].y - self.measurements['start_pos'].y
        self.distance_traveled = math.sqrt(dx*dx + dy*dy)
        
        if self.distance_traveled >= self.straight_line:
            self.phase = "TURN"
            self.measurements['turn_start_pos'] = self.measurements['current_pos']
            rospy.loginfo("Starting turn phase")

    def update_turn_measurements(self):
        if not self.measurements['turn_start_pos']:
            return
            
        dx = self.measurements['current_pos'].x - self.measurements['turn_start_pos'].x
        dy = self.measurements['current_pos'].y - self.measurements['turn_start_pos'].y
        
        # Update advance and transfer
        self.measurements['advance'] = abs(dx)
        self.measurements['transfer'] = abs(dy)
        
        if not self.measurements['turn_complete'] and self.measurements['advance'] >= self.max_advance:
            self.measurements['turn_complete'] = True
            self.phase = "STOP"
            rospy.loginfo(f"Turn complete:")
            rospy.loginfo(f"  Advance: {self.measurements['advance']:.2f}m")
            rospy.loginfo(f"  Transfer: {self.measurements['transfer']:.2f}m")
            rospy.loginfo(f"  Tactical Diameter: {self.measurements['tactical_diameter']:.2f}m")

    def check_stop_condition(self):
        dx = self.measurements['current_pos'].x - self.measurements['turn_start_pos'].x
        dy = self.measurements['current_pos'].y - self.measurements['turn_start_pos'].y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance >= self.stop_distance:
            rospy.loginfo("Maneuver complete - stopping WAM-V")
            self.cruise_speed = 0.0

    def run(self):
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            cmd = Twist()
            
            if self.phase == "STRAIGHT":
                cmd.linear.x = self.cruise_speed + self.speed_control
                cmd.angular.z = 0.0
            elif self.phase == "TURN":
                cmd.linear.x = self.cruise_speed + self.speed_control
                cmd.angular.z = self.target_turn_rate + self.turn_rate_control
            elif self.phase == "STOP":
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurningCircleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
