#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class TurningCircleController:
    def __init__(self):
        rospy.init_node('turning_circle_controller')
        
        # Fixed parameters
        self.boat_length = 4.06
        self.scenario = rospy.get_param('~scenario', 'three_times')
        self.turn_direction = rospy.get_param('~turn_direction', 'port')
        
        # IMO Rudder Standards
        self.max_rudder_angle = 35  # degrees
        self.initial_turn_angle = 15  # degrees
        self.steady_turn_angle = 25  # degrees
        self.max_rudder_rate = 2.32  # degrees/second
        
        # Convert to radians
        self.max_rudder_rad = math.radians(self.max_rudder_angle)
        self.initial_turn_rad = math.radians(self.initial_turn_angle)
        self.steady_turn_rad = math.radians(self.steady_turn_angle)
        self.max_rate_rad = math.radians(self.max_rudder_rate)
        
        # Scenario specific parameters
        if self.scenario == 'three_times':
            self.straight_line_length = self.boat_length * 3
            self.tactical_diameter = self.boat_length * 3
        else:
            self.straight_line_length = self.boat_length * 2
            self.tactical_diameter = self.boat_length * 2

        # Calculate turn radius and rates
        self.turn_radius = self.tactical_diameter / 2
        self.cruise_speed = 3.0
        base_turn_rate = (self.cruise_speed / self.turn_radius)
        self.turn_rate = base_turn_rate if self.turn_direction == 'port' else -base_turn_rate
        
        # State variables
        self.phase = 'STRAIGHT'
        self.distance_traveled = 0.0
        self.turn_angle = 0.0
        self.start_pos = None
        self.turn_start_pos = None
        self.last_heading = None
        self.total_angle_turned = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Speed control topics
        self.speed_setpoint_pub = rospy.Publisher('/speed/setpoint', Float64, queue_size=1)
        self.speed_state_pub = rospy.Publisher('/speed/state', Float64, queue_size=1)
        self.speed_control_sub = rospy.Subscriber('/speed/control_effort', Float64, self.speed_control_callback)
        self.speed_enable_pub = rospy.Publisher('/speed/pid_enable', Bool, queue_size=1)
        
        # Heading control topics
        self.heading_setpoint_pub = rospy.Publisher('/heading/setpoint', Float64, queue_size=1)
        self.heading_state_pub = rospy.Publisher('/heading/state', Float64, queue_size=1)
        self.heading_control_sub = rospy.Subscriber('/heading/control_effort', Float64, self.heading_control_callback)
        self.heading_enable_pub = rospy.Publisher('/heading/pid_enable', Bool, queue_size=1)
        
        # Turn rate control topics
        self.turn_rate_setpoint_pub = rospy.Publisher('/turn_rate/setpoint', Float64, queue_size=1)
        self.turn_rate_state_pub = rospy.Publisher('/turn_rate/state', Float64, queue_size=1)
        self.turn_rate_control_sub = rospy.Subscriber('/turn_rate/control_effort', Float64, self.turn_rate_control_callback)
        self.turn_rate_enable_pub = rospy.Publisher('/turn_rate/pid_enable', Bool, queue_size=1)
        
        # Control values
        self.speed_control = 0.0
        self.heading_control = 0.0
        self.turn_rate_control = 0.0
        
        # Set up control rate
        self.rate = rospy.Rate(10)
        
        rospy.loginfo(f"Initialized for {self.scenario} scenario with {self.turn_direction} turn")
        rospy.loginfo(f"Target Tactical Diameter: {self.tactical_diameter}m")
        rospy.loginfo(f"Turn Radius: {self.turn_radius}m")
        rospy.loginfo(f"Turn Rate: {math.degrees(abs(self.turn_rate))} deg/s")

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_yaw(self, orientation):
        """Extract yaw from quaternion"""
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

    def update_turn_angle(self, current_heading):
        if self.last_heading is not None:
            diff = self.normalize_angle(current_heading - self.last_heading)
            self.total_angle_turned += diff
        self.last_heading = current_heading

    def speed_control_callback(self, msg):
        self.speed_control = msg.data

    def heading_control_callback(self, msg):
        self.heading_control = msg.data

    def turn_rate_control_callback(self, msg):
        self.turn_rate_control = msg.data

    def update_measurements(self, current_pos):
        """Update measurements during turning phase"""
        if self.turn_start_pos is not None:
            dx = current_pos.x - self.turn_start_pos.x
            dy = current_pos.y - self.turn_start_pos.y
            current_diameter = 2 * math.sqrt(dx*dx + dy*dy)
            
            angle_deg = math.degrees(abs(self.total_angle_turned))
            
            if 89 <= angle_deg <= 91 and not hasattr(self, 'logged_90'):
                rospy.loginfo(f"=== 90° Turn Measurements ===")
                rospy.loginfo(f"Advance: {abs(dx):.2f}m")
                rospy.loginfo(f"Transfer: {abs(dy):.2f}m")
                self.logged_90 = True
                
            if 179 <= angle_deg <= 181 and not hasattr(self, 'logged_180'):
                rospy.loginfo(f"=== 180° Turn Measurements ===")
                rospy.loginfo(f"Tactical Diameter: {current_diameter:.2f}m")
                self.logged_180 = True
                
            if 359 <= angle_deg <= 361 and not hasattr(self, 'logged_360'):
                rospy.loginfo(f"=== 360° Turn Measurements ===")
                rospy.loginfo(f"Final Diameter: {current_diameter:.2f}m")
                rospy.loginfo(f"Total Angle: {angle_deg:.1f}°")
                self.logged_360 = True

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('wamv')
            pose = msg.pose[idx]
            twist = msg.twist[idx]
            
            current_pos = pose.position
            current_heading = self.get_yaw(pose.orientation)
            current_speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
            current_turn_rate = twist.angular.z
            
            # Initialize start position
            if self.start_pos is None:
                self.start_pos = current_pos
                self.last_heading = current_heading
            
            # Publish current states for PID control
            self.speed_state_pub.publish(Float64(current_speed))
            self.heading_state_pub.publish(Float64(current_heading))
            self.turn_rate_state_pub.publish(Float64(current_turn_rate))
            
            # Update phase and publish setpoints
            if self.phase == 'STRAIGHT':
                self.distance_traveled = math.sqrt(
                    (current_pos.x - self.start_pos.x)**2 + 
                    (current_pos.y - self.start_pos.y)**2
                )
                
                # Publish setpoints for straight line
                self.speed_setpoint_pub.publish(Float64(self.cruise_speed))
                self.heading_setpoint_pub.publish(Float64(self.last_heading))
                self.turn_rate_setpoint_pub.publish(Float64(0.0))
                
                # Check for phase transition
                if self.distance_traveled >= self.straight_line_length:
                    self.phase = 'TURN'
                    self.turn_start_pos = current_pos
                    self.total_angle_turned = 0.0
                    rospy.loginfo(f"=== Starting {self.turn_direction.capitalize()} Turn ===")
                    rospy.loginfo(f"Turn Start Position: x={current_pos.x:.2f}, y={current_pos.y:.2f}")
                    
            elif self.phase == 'TURN':
                if self.turn_start_pos is None:
                    self.turn_start_pos = current_pos
                
                self.update_turn_angle(current_heading)
                
                # Publish setpoints for turning
                self.speed_setpoint_pub.publish(Float64(self.cruise_speed))
                turn_direction_multiplier = 1.0 if self.turn_direction == 'port' else -1.0
                desired_heading = self.last_heading + (turn_direction_multiplier * math.pi/2)
                self.heading_setpoint_pub.publish(Float64(desired_heading))
                self.turn_rate_setpoint_pub.publish(Float64(self.turn_rate))
                
                # Calculate and log measurements
                self.update_measurements(current_pos)
                
        except ValueError:
            rospy.logwarn_throttle(5, "WAM-V not found in model states")

    def run(self):
        # Enable all PID controllers
        self.speed_enable_pub.publish(Bool(True))
        self.heading_enable_pub.publish(Bool(True))
        self.turn_rate_enable_pub.publish(Bool(True))
        
        while not rospy.is_shutdown():
            cmd = Twist()
            
            if self.phase == 'STRAIGHT':
                cmd.linear.x = self.speed_control
                cmd.angular.z = self.heading_control
            else:  # TURN phase
                cmd.linear.x = self.speed_control
                cmd.angular.z = self.turn_rate_control  # Use turn rate control during turns
            
            self.cmd_vel_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurningCircleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
