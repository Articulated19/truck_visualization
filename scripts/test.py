#!/usr/bin/env python
import rospy
import math
import tf

#from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker

if __name__=="__main__":
    rospy.init_node('test_marker', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    box = Marker()

    box.header.frame_id = "/my_frame"
    box.header.stamp = rospy.Time.now()

    box.ns = "box"
    box.id = 1

    box.type = Marker.CUBE
    box.action = Marker.ADD

    box.lifetime = rospy.Duration(0)
    
    box.scale.x = 1.0
    box.scale.y = 1.0
    box.scale.z = 1.0

    box.color.r = 0.5
    box.color.g = 0.5
    box.color.b = 0.0
    box.color.a = 1.0
    
    box.pose.position.x = 0.0
    box.pose.position.y = 0.0
    box.pose.position.z = 0.0
    
    quat = tf.transformations.quaternion_from_euler(0, 0, math.pi/4)
    box.pose.orientation.x = quat[0]
    box.pose.orientation.y = quat[1]
    box.pose.orientation.z = quat[2]
    box.pose.orientation.w = quat[3]

    while not rospy.is_shutdown():
        pub.publish(box)
        rospy.sleep(0.02)
    




