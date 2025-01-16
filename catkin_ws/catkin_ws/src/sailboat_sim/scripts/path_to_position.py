#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

class PathToPositionConverter:
    def __init__(self):
        rospy.init_node('path_to_position_converter')
        
        # Publishers for actual path coordinates
        self.actual_x_pub = rospy.Publisher('/wamv/actual_path/x', Point, queue_size=10)
        self.actual_y_pub = rospy.Publisher('/wamv/actual_path/y', Point, queue_size=10)
        
        # Publishers for simulated path coordinates
        self.sim_x_pub = rospy.Publisher('/wamv/simulated_path/x', Point, queue_size=10)
        self.sim_y_pub = rospy.Publisher('/wamv/simulated_path/y', Point, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/wamv/actual_path', Path, self.actual_path_callback)
        rospy.Subscriber('/wamv/simulated_path', Path, self.simulated_path_callback)

    def actual_path_callback(self, path_msg):
        if path_msg.poses:
            point = Point()
            point.x = path_msg.poses[-1].pose.position.x
            point.y = path_msg.poses[-1].pose.position.y
            point.z = 0.0
            
            self.actual_x_pub.publish(point)
            self.actual_y_pub.publish(point)

    def simulated_path_callback(self, path_msg):
        if path_msg.poses:
            point = Point()
            point.x = path_msg.poses[-1].pose.position.x
            point.y = path_msg.poses[-1].pose.position.y
            point.z = 0.0
            
            self.sim_x_pub.publish(point)
            self.sim_y_pub.publish(point)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = PathToPositionConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
