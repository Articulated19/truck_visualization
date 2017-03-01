#!/usr/bin/env python
import rospy
import math
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=True)
        rospy.Subscriber('sim_pose', Pose, self.poseCallback)
        self.pub = rospy.Publisher('truck_marker', Marker, queue_size=10)
        self.truck = TruckModel()

        while not rospy.is_shutdown():
            self.pub.publish(self.truck.getMarker())
            print self.truck.getMarker()
            rospy.sleep(0.02)

    def poseCallback(self, msg):
        self.truck.setPose(msg)

class TruckModel:
    def __init__(self):
        self.header = Marker()
        self.header.header.frame_id = "/truck"
        self.header.header.stamp = rospy.Time.now()

        self.header.ns = "truck"
        self.header.id = 1

        self.header.type = Marker.CUBE
        self.header.action = Marker.ADD

        self.header.lifetime = rospy.Duration(0)
        
        # Truck measurments in meters
        self.header.scale.x = 420
        self.header.scale.y = 190
        self.header.scale.z = 220

        self.header.color.r = 1.0
        self.header.color.g = 0.0
        self.header.color.b = 0.0
        self.header.color.a = 1.0
        
        self.header.pose.position.x = 0.0
        self.header.pose.position.y = 0.0
        self.header.pose.position.z = 0.11
        
        self.setDirection(45)

    def getMarker(self):
        return self.header

    def setPose(self, pose):
        self.header.pose = pose

    def setPosition(self, position):
        self.header.pose.position = position

    # Takes angle in degrees and sets the direction of the marker
    def setDirection(self, angle):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(angle))
        self.header.pose.orientation.x = quat[0]
        self.header.pose.orientation.y = quat[1]
        self.header.pose.orientation.z = quat[2]
        self.header.pose.orientation.w = quat[3]

if __name__=="__main__":
    Visualizer()
    