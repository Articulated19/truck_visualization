#!/usr/bin/env python
import rospy
import math
import tf
import os
from threading import Thread
from os.path import dirname, abspath
import numpy as np
import sys
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from custom_msgs.msg import TruckState, Position
import custom_msgs.msg as cm 
import cv2

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *


#lib_path = os.path.abspath(os.path.join('..', '..', '..', 'lib'))
#sys.path.append(lib_path)

#import mymodule

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        
        self.goals = []
        
        self.truck = TruckModel()
        self.map_obj = Map()
        self.mapmodel = MapModel(self.map_obj)
        
        
        self.width = self.mapmodel.width * self.mapmodel.resolution
        self.height = self.mapmodel.height * self.mapmodel.resolution
        
        rospy.Subscriber('truck_state', TruckState, self.stateCallback)
        rospy.Subscriber('rviz_path', cm.Path, self.pathCallback)
        
        rospy.Subscriber('alg_startend',cm.Path, self.algStartEndCallback)
        
        rospy.Subscriber('long_path', cm.Path, self.longPathCallback)
        rospy.Subscriber('ref_path', cm.Path, self.refPathCallback)
        
        rospy.Subscriber('clicked_point', PointStamped, self.pointClickedCallback)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goalPointCallback)
        
        
        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)
        
        
        self.goal_pub = rospy.Publisher('truck_goals', cm.Path, queue_size=10)
        self.truck_pub = rospy.Publisher('truck_marker', Marker, queue_size=10)
        self.map_pub = rospy.Publisher("truck_map", OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher("truck_path", Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher("truck_ref_path", Path, queue_size=10)
        self.long_path_pub = rospy.Publisher("truck_long_path", Path, queue_size=10)
        self.endpoint_pub = rospy.Publisher("end_point", PointStamped, queue_size=10)
        self.startpoint_pub = rospy.Publisher("start_point", PointStamped, queue_size=10)
        
        rospy.sleep(2)
        self.map_pub.publish(self.mapmodel.map)
        
    def algStartEndCallback(self, data):
        p = data.path
        
        s = PointStamped()
        s.header.frame_id = "truck"
        s.point.x = p[0].x
        s.point.y = self.height - p[0].y
        
        self.startpoint_pub.publish(s)
        
        e = PointStamped()
        e.header.frame_id = "truck"
        e.point.x = p[1].x
        e.point.y = self.height - p[1].y
        
        self.endpoint_pub.publish(e)
        
        
    
    def mapUpdateHandler(self, data):
        obst = data.data
        add = self.map_obj.addObstacle(obst)
        if not add:
            rem = self.map_obj.removeObstacle(obst)
            if not rem:
                print "can't add or remove obstacle"
                
        self.mapmodel = MapModel(self.map_obj)
        self.map_pub.publish(self.mapmodel.map)

        
    def pointClickedCallback(self, data):
        p = data.point
        self.goals.append((p.x, p.y))
        
    def goalPointCallback(self, data):
        p = data.pose.position
        self.goals.append((p.x, p.y))
        
        gm = [cm.Position(x,self.height - y) for x,y in self.goals]
        self.goal_pub.publish(gm)
        self.goals = []
    
    def stateCallback(self, msg):
        x = msg.p.x
        y = msg.p.y
        theta1 = msg.theta1
        theta2 = msg.theta2
        
        self.truck.setHeaderPosition(x - self.truck.hl/2 * math.cos(theta1), \
                self.height - (y- self.truck.hl/2 * math.sin(theta1)))
        self.truck.setHeaderDirection(-theta1)
        
        jx, jy = (x - self.truck.hl * math.cos(theta1), y- self.truck.hl * math.sin(theta1))
        mtx, mty = jx - 0.5*self.truck.tl * math.cos(theta2), jy - 0.5*self.truck.tl * math.sin(theta2)
        
        
        
        
        self.truck.setTrailerPosition(mtx, self.height - mty)
        self.truck.setTrailerDirection(-theta2)
    
        self.truck_pub.publish(self.truck.header)
        self.truck_pub.publish(self.truck.trailer)
        
    def pathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]
        
        p = TruckPath(path, 30)
        self.path_pub.publish(p.path_msg)
        
    def refPathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]
        
        p = TruckPath(path, 10)
        self.ref_path_pub.publish(p.path_msg)
        
    def longPathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]
        
        p = TruckPath(path, 20)
        self.long_path_pub.publish(p.path_msg)

  
        
class TruckPath:
    def __init__(self, path, z):
        

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'truck'

        for x,y in path:
            posestmpd = PoseStamped()

            posestmpd.header.frame_id = 'truck'

            posestmpd.pose.position.x = x
            posestmpd.pose.position.y = 9650-y
            posestmpd.pose.position.z = z

            posestmpd.pose.orientation.w = 0.0
            posestmpd.pose.orientation.x = 0.0
            posestmpd.pose.orientation.y = 0.0
            posestmpd.pose.orientation.z = 0.0

            self.path_msg.poses.append(posestmpd)
        
        

class TruckModel:
    def __init__(self):
        
        self.hl = 270
        self.hw = 180
        
        self.tl = 620
        self.tw = 180
        
        
        # Create the header marker, and set a static frame
        self.header = Marker()
        self.header.header.frame_id = "truck"
        #self.header.header.stamp = rospy.Time.now()

        # Set the namespace and id of the header, making it unique
        self.header.ns = "truck"
        self.header.id = 1

        self.header.type = Marker.CUBE
        self.header.action = Marker.ADD

        # 0 means that header is now immortal
        self.header.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.header.scale.x = self.hl #420
        self.header.scale.y = self.hw
        self.header.scale.z = 220

        # sick blue color
        self.header.color.r = 0.1294117647
        self.header.color.g = 0.58823529411
        self.header.color.b = 0.95294117647
        self.header.color.a = 1.0
        
        #self.header.pose.position.x = 0.0
        #self.header.pose.position.y = 0.0
        self.header.pose.position.z = 110

        self.trailer = Marker()
        self.trailer.header.frame_id = "truck"
        #self.trailer.header.stamp = rospy.Time.now()

        self.trailer.ns = "trailer"
        self.trailer.id = 1

        self.trailer.type = Marker.CUBE
        self.trailer.action = Marker.ADD

        # 0 means that trailer is now immortal
        self.trailer.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.trailer.scale.x = self.tl
        self.trailer.scale.y = self.tw
        self.trailer.scale.z = 220

        # sick red color
        self.trailer.color.r = 0.956862745
        self.trailer.color.g = 0.26274509803
        self.trailer.color.b = 0.21176470588
        self.trailer.color.a = 1.0
        
        #self.trailer.pose.position.x = 0.0
        #self.trailer.pose.position.y = -self.trailer.scale.x
        self.trailer.pose.position.z = 110
        
        #self.setHeaderDirection(math.pi/4+0.4)

   
    def setHeaderPosition(self, x, y):
        self.header.pose.position.x = x
        self.header.pose.position.y = y
        
    def setTrailerPosition(self, x, y):
        self.trailer.pose.position.x = x
        self.trailer.pose.position.y = y

    def setTrailerDirection(self, angle):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        self.trailer.pose.orientation.x = quat[0]
        self.trailer.pose.orientation.y = quat[1]
        self.trailer.pose.orientation.z = quat[2]
        self.trailer.pose.orientation.w = quat[3]
    
    # Takes angle in degrees and sets the direction of the marker
    def setHeaderDirection(self, angle):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        self.header.pose.orientation.x = quat[0]
        self.header.pose.orientation.y = quat[1]
        self.header.pose.orientation.z = quat[2]
        self.header.pose.orientation.w = quat[3]

class MapModel:
    def __init__(self, map_obj):
        
        stamp = rospy.Time.now()

        self.map = OccupancyGrid()
        self.map.header.frame_id = "truck"
        self.map.header.stamp = stamp

        # The time at which the map was loaded
        load_time = stamp
        
        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        origin = Pose(Point(0,0,0), Quaternion(0,0,0,0))

        mapFromImg, self.resolution = map_obj.getMapAndScale()
        
        mapFromImg = mapFromImg[::-1]
        
        self.width = len(mapFromImg[0])
        self.height = len(mapFromImg)
        
        self.map.info = MapMetaData(load_time, self.resolution, self.width, self.height, origin)
        
        for row in range(self.height):
            for col in range(self.width):
                c = mapFromImg[row][col]
                if c == 0:
                    self.map.data.append(100)
                elif c == 1:
                    self.map.data.append(0)
                    
                else:
                    self.map.data.append(50)

        
        
        
    
         

if __name__=="__main__":
    Visualizer()
    rospy.spin()
    
