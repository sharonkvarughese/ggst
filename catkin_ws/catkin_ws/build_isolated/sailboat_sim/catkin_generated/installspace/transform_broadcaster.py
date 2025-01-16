#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class FrameBroadcaster:
    def __init__(self):
        rospy.init_node('world_frame_broadcaster')
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transforms = []
        
        # map -> world transform
        t1 = TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "map"
        t1.child_frame_id = "world"
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        self.static_transforms.append(t1)
        
        # world -> odom transform
        t2 = TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "world"
        t2.child_frame_id = "odom"
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.static_transforms.append(t2)
        
    def run(self):
        self.static_broadcaster.sendTransform(self.static_transforms)
        rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster = FrameBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass
