#!/usr/bin/env python
import rospy
import math
import tf
import os

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=True)
        rospy.Subscriber('sim_pose', Pose, self.poseCallback)
        self.pub = rospy.Publisher('truck_marker', Marker, queue_size=10)
        self.truck = TruckModel()
        self.path = TruckPath()
        rospy.spin()

    def poseCallback(self, msg):
        self.truck.setPose(msg)
        markers = self.truck.getMarkers()
        self.pub.publish(markers[0])
        self.pub.publish(markers[1])
        self.path.pub.publish(self.path.path)
        
class TruckPath:
    def __init__(self):
        self.points = [(0,0,0)]
        self.pub = rospy.Publisher('sim_path', Path, queue_size=10)
        self.createPath('/../path.txt')

        self.path = Path()
        self.path.header.frame_id = 'truck'
        self.path.header.stamp = rospy.Time.now()
        id = 0
        for m in self.points:
            posestmpd = PoseStamped()

            posestmpd.header.seq = id
            posestmpd.header.frame_id = 'truck'
            posestmpd.header.stamp = rospy.Time.now()

            posestmpd.pose.position.x = m[0]
            posestmpd.pose.position.y = m[1]
            posestmpd.pose.position.z = m[2]

            posestmpd.pose.orientation.w = 0.0

            self.path.poses.append(posestmpd)
            id += 1
        
        print self.path
    
    def createPath(self, path):
        dirpath = os.path.dirname(os.path.abspath(__file__))
        f = open(dirpath+path, 'r')
        lines = [line.rstrip('\n') for line in f.readlines()]
        posL = [s.split(' ', 1 ) for s in lines]
        for l in posL:
            self.points.append((int(l[0]),int(l[1]),0))

class TruckModel:
    def __init__(self):
        # Create the header marker, and set a static frame
        self.header = Marker()
        self.header.header.frame_id = "/truck"
        self.header.header.stamp = rospy.Time.now()

        # Set the namespace and id of the header, making it unique
        self.header.ns = "truck"
        self.header.id = 1

        self.header.type = Marker.CUBE
        self.header.action = Marker.ADD

        # 0 means that header is now immortal
        self.header.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.header.scale.x = 420 #420
        self.header.scale.y = 190
        self.header.scale.z = 220

        self.header.color.r = 0.75
        self.header.color.g = 0.75
        self.header.color.b = 0.75
        self.header.color.a = 1.0
        
        self.header.pose.position.x = 0.0
        self.header.pose.position.y = 0.0
        self.header.pose.position.z = 110

        self.trailer = Marker()
        self.trailer.header.frame_id = "/truck"
        self.trailer.header.stamp = rospy.Time.now()

        self.trailer.ns = "trailer"
        self.trailer.id = 1

        self.trailer.type = Marker.CUBE
        self.trailer.action = Marker.ADD

        # 0 means that trailer is now immortal
        self.trailer.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.trailer.scale.x = 420
        self.trailer.scale.y = 190
        self.trailer.scale.z = 220

        self.trailer.color.r = 0.75
        self.trailer.color.g = 0.75
        self.trailer.color.b = 0.75
        self.trailer.color.a = 1.0
        
        self.trailer.pose.position.x = 0.0
        self.trailer.pose.position.y = -self.trailer.scale.x
        self.trailer.pose.position.z = 110
        
        self.setDirection(45)

    def getMarkers(self):
        return [self.header, self.trailer]

    def setPose(self, pose):
        theta = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
        pose.position.x = pose.position.x + (self.header.scale.x/2) * math.cos(theta)
        pose.position.y = pose.position.y + (self.header.scale.x/2) * math.sin(theta)
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
    