#!/usr/bin/env python
import rospy
import math
import tf
import os
from threading import Thread
from os.path import dirname, abspath
import numpy as np
import cv2

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from custom_msgs.msg import TruckState, Position
import custom_msgs.msg as cm 

MAP_PATH = "/map.png"

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        self.truck = TruckModel()
        self.path = TruckPath()
        self.map = MapModel()
        # rospy.Subscriber('sim_header_pose', Pose, self.headerPoseCallback)
        # rospy.Subscriber('sim_trailer_pose', Pose, self.trailerPoseCallback)
        rospy.Subscriber('truck_state', TruckState, self.stateCallback)
        self.pub = rospy.Publisher('truck_marker', Marker, queue_size=10)
        rospy.spin()

    def stateCallback(self, msg):
        pose = self.posToPose(msg.p, msg.theta1)
        self.truck.setHeaderPose(pose)

        trailerpose = self.poseToTrailerPose(pose, 1000, msg.theta2)
        self.truck.setTrailerPose(trailerpose)

        markers = self.truck.getMarkers()

        self.pub.publish(markers[0])
        self.pub.publish(markers[1])
        self.path.pub.publish(self.path.path)
        # self.map.pub.publish(self.map.map)

    # def trailerPoseCallback(self, msg):
    #     self.truck.setTrailerPose(msg)
    #     markers = self.truck.getMarkers()
    #     self.pub.publish(markers[1])
    
    # def headerPoseCallback(self, msg):
    #     self.truck.setHeaderPose(msg)
    #     markers = self.truck.getMarkers()
    #     self.pub.publish(markers[0])

    def posToPose(self, pos, theta):
        q = self.thetaToQuat(theta)
        p = Point(pos.x, pos.y, 110)
        pose = Pose(p, q)
        return pose

    def poseToTrailerPose(self, pose, trailerlength, theta):
        pose.position.x = pose.position.x - trailerlength * math.cos(theta)
        pose.position.y = pose.position.y + trailerlength * math.sin(theta)
        pose.orientation = self.thetaToQuat(theta)
        return pose
    
    def thetaToQuat(self, theta):
        quat = tf.transformations.quaternion_about_axis(theta, (0,0, 1))
        return Quaternion(quat[0], quat[1], quat[2], quat[3])
        
class TruckPath:
    def __init__(self):
        self.points = []
        self.pub = rospy.Publisher('sim_path', Path, queue_size=10)
        rospy.Subscriber('rviz_path', cm.Path, self.pathCallback)
        # self.createPath('/../path.txt')

        self.path = Path()
        self.path.header.frame_id = 'truck'
        self.path.header.stamp = rospy.Time.now()
        id = 0

    def pathCallback(self, msg):
        self.path = Path()
        self.path.header.frame_id = 'truck'
        self.path.header.stamp = rospy.Time.now()
        for m in msg.path:
            posestmpd = PoseStamped()

            # posestmpd.header.seq = id
            posestmpd.header.frame_id = 'truck'
            posestmpd.header.stamp = rospy.Time.now()

            posestmpd.pose.position.x = m.x
            posestmpd.pose.position.y = m.y
            posestmpd.pose.position.z = 30

            posestmpd.pose.orientation.w = 0.0

            self.path.poses.append(posestmpd)
            # id += 1
        while not rospy.is_shutdown():
            self.pub.publish(self.path)
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
        self.header.header.frame_id = "truck"
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

        # sick blue color
        self.header.color.r = 0.1294117647
        self.header.color.g = 0.58823529411
        self.header.color.b = 0.95294117647
        self.header.color.a = 1.0
        
        self.header.pose.position.x = 0.0
        self.header.pose.position.y = 0.0
        self.header.pose.position.z = 110

        self.trailer = Marker()
        self.trailer.header.frame_id = "truck"
        self.trailer.header.stamp = rospy.Time.now()

        self.trailer.ns = "trailer"
        self.trailer.id = 1

        self.trailer.type = Marker.CUBE
        self.trailer.action = Marker.ADD

        # 0 means that trailer is now immortal
        self.trailer.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.trailer.scale.x = 1000
        self.trailer.scale.y = 190
        self.trailer.scale.z = 220

        # sick red color
        self.trailer.color.r = 0.956862745
        self.trailer.color.g = 0.26274509803
        self.trailer.color.b = 0.21176470588
        self.trailer.color.a = 1.0
        
        self.trailer.pose.position.x = 0.0
        self.trailer.pose.position.y = -self.trailer.scale.x
        self.trailer.pose.position.z = 110
        
        self.setDirection(45)

    def getMarkers(self):
        return [self.header, self.trailer]

    def setHeaderPose(self, pose):
        # theta = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
        # pose.position.x = pose.position.x + (self.header.scale.x/2) * math.cos(theta)
        # pose.position.y = pose.position.y + (self.header.scale.x/2) * math.sin(theta)
        self.header.pose = pose

    def setTrailerPose(self, pose):
        # theta = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
        # pose.position.x = pose.position.x + (self.trailer.scale.x/2) * math.cos(theta)
        # pose.position.y = pose.position.y + (self.trailer.scale.x/2) * math.sin(theta)
        self.trailer.pose = pose

    def setPosition(self, position):
        self.header.pose.position = position

    # Takes angle in degrees and sets the direction of the marker
    def setDirection(self, angle):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(angle))
        self.header.pose.orientation.x = quat[0]
        self.header.pose.orientation.y = quat[1]
        self.header.pose.orientation.z = quat[2]
        self.header.pose.orientation.w = quat[3]

class MapModel:
    def __init__(self):
        stamp = rospy.Time.now()

        self.map = OccupancyGrid()
        self.map.header.frame_id = "truck"
        self.map.header.stamp = stamp

        # The time at which the map was loaded
        load_time = stamp
        # The map resolution [m/cell]
        resolution = 10
        width = 490
        height = 965
        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        origin = Pose(Point(0,0,0), Quaternion(0,0,0,0))

        self.map.info = MapMetaData(load_time, resolution, width, height, origin)

        mapFromImg = self.readImgToMatrix(MAP_PATH)
        for row in range(len(mapFromImg)):
            for elem in range(len(mapFromImg[row])):
                self.map.data.append(mapFromImg[row][elem])

        self.pub = rospy.Publisher("truck_map", OccupancyGrid, queue_size=10)
        self.pub.publish(self.map)
        self.rate = rospy.Rate(1)
        # thread = Thread(None, self.loop, "loop")
        # thread.start()
    
    def loop(self):
        while not rospy.is_shutdown() :
            self.pub.publish(self.map)
            self.rate.sleep()

    def readImgToMatrix(self, path):
        dirpath = dirname(abspath(__file__))
        matrix = np.asarray(cv2.imread(dirpath + path, 0), dtype=np.uint8).tolist()

        # Going through all elements, adjusting their value to match [0, 1, 2]
        # Where:
        #     0 = Black
        #     1 = White
        #     2 = Grey
        for row in range(len(matrix)):
            for elem in range(len(matrix[row])):
                # Not black
                if matrix[row][elem] != 0:
                    # White
                    if matrix[row][elem] == 255:
                        matrix[row][elem] = 50
                    # Grey
                    else:
                        matrix[row][elem] = 100

        #plt.imshow(matrix)
        #plt.show()
        return matrix
         

if __name__=="__main__":
    Visualizer()
    