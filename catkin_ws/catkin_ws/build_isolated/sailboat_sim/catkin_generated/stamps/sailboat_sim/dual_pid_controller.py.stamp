#!/usr/bin/env python3
import rospy
import math
import xml.etree.ElementTree as ET
import os
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import rospkg

class CircularTrajectoryController:
    def __init__(self):
        rospy.init_node('wamv_circular_trajectory')
        
        # Get WAM-V length from SDF
        self.boat_length = self.get_wamv_length()
        rospy.loginfo(f"WAM-V length from SDF: {self.boat_length} meters")
        
        # Get scenario selection
        self.scenario = rospy.get_param('~scenario', 'twice')  # 'twice' or 'thrice'
        if self.scenario == 'twice':
            self.diameter = self.boat_length * 2
        else:
            self.diameter = self.boat_length * 3
        
        self.radius = self.diameter / 2.0
        self.linear_speed = rospy.get_param('~linear_speed', 10.0)  # m/s
        
        # Initialize states
        self.current_position = None
        self.current_heading = None
        self.target_heading = 0.0
        
        # Publishers for PID controllers
        self.angular_setpoint_pub = rospy.Publisher('heading_controller/setpoint', Float64, queue_size=1)
        self.angular_state_pub = rospy.Publisher('heading_controller/state', Float64, queue_size=1)
        
        self.radial_setpoint_pub = rospy.Publisher('radius_controller/setpoint', Float64, queue_size=1)
        self.radial_state_pub = rospy.Publisher('radius_controller/state', Float64, queue_size=1)
        
        # Subscribe to PID control efforts
        rospy.Subscriber('heading_controller/control_effort', Float64, self.heading_control_callback)
        rospy.Subscriber('radius_controller/control_effort', Float64, self.radius_control_callback)
        
        # Subscribe to WAM-V pose
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Control variables
        self.angular_control = 0.0
        self.radial_control = 0.0
        
        rospy.loginfo(f"Circular trajectory controller initialized for {self.scenario} boat length diameter")
        rospy.loginfo(f"Circle diameter: {self.diameter:.2f}m, radius: {self.radius:.2f}m")
        rospy.loginfo(f"Target linear speed: {self.linear_speed}m/s")
        
        self.rate = rospy.Rate(50)  # 50 Hz control loop

    def get_wamv_length(self):
        """Extract WAM-V length from SDF file"""
        try:
            # Get the path to the WAM-V SDF file
            rospack = rospkg.RosPack()
            sdf_path = os.path.join(rospack.get_path('sailboat_sim'), 'models', 'wam-v', 'model.sdf')
            
            # Parse the SDF file
            tree = ET.parse(sdf_path)
            root = tree.getroot()
            
            # Look for collision or visual geometry
            # First try to find the hull base length
            for link in root.findall(".//link"):
                for collision in link.findall(".//collision"):
                    size = collision.find(".//size")
                    if size is not None:
                        # Assuming the length is the first dimension in size
                        length = float(size.text.split()[0])
                        return length
                        
            # If not found in collision, try visual geometry
            for link in root.findall(".//link"):
                for visual in link.findall(".//visual"):
                    size = visual.find(".//size")
                    if size is not None:
                        # Assuming the length is the first dimension in size
                        length = float(size.text.split()[0])
                        return length
                        
            # If no explicit size found, use default WAM-V length
            rospy.logwarn("Could not find WAM-V length in SDF, using default value of 5.0 meters")
            return 5.0
            
        except Exception as e:
            rospy.logerr(f"Error reading WAM-V length from SDF: {str(e)}")
            rospy.logwarn("Using default WAM-V length of 5.0 meters")
            return 5.0
            
    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('wamv')
            pose = msg.pose[idx]
            
            # Get position
            self.current_position = pose.position
            
            # Get heading from quaternion
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            )
            _, _, self.current_heading = euler_from_quaternion(quaternion)
            
            # Calculate radial distance from origin
            current_radius = math.sqrt(
                self.current_position.x ** 2 + 
                self.current_position.y ** 2
            )
            
            # Calculate target heading (tangent to circle)
            self.target_heading = math.atan2(
                self.current_position.x,
                -self.current_position.y
            )
            
            # Publish current states to PID controllers
            self.angular_state_pub.publish(Float64(self.current_heading))
            self.angular_setpoint_pub.publish(Float64(self.target_heading))
            
            self.radial_state_pub.publish(Float64(current_radius))
            self.radial_setpoint_pub.publish(Float64(self.radius))
            
        except ValueError:
            rospy.logwarn_throttle(5, "WAM-V model not found in model states")
            
    def heading_control_callback(self, msg):
        self.angular_control = msg.data
        
    def radius_control_callback(self, msg):
        self.radial_control = msg.data
        
    def run(self):
        while not rospy.is_shutdown():
            if self.current_position is not None:
                try:
                    # Create velocity command
                    cmd_vel = Twist()
                    
                    # Base linear speed plus radial correction
                    cmd_vel.linear.x = self.linear_speed + self.radial_control
                    
                    # Angular velocity from heading PID
                    cmd_vel.angular.z = self.angular_control
                    
                    # Publish command
                    self.cmd_vel_pub.publish(cmd_vel)
                    
                    rospy.loginfo_throttle(1, 
                        f"linear_speed: {cmd_vel.linear.x:.2f} m/s, "
                        f"angular_speed: {cmd_vel.angular.z:.2f} rad/s"
                    )
                    
                except Exception as e:
                    rospy.logerr(f"Error in control loop: {str(e)}")
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = CircularTrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
