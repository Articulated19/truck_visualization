#!/usr/bin/env python
import rospy
import math
import tf
import os
from threading import Thread
from os.path import dirname, abspath
import numpy as np
import sys
from std_msgs.msg import Int8, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from custom_msgs.msg import TruckState, Position
import custom_msgs.msg as cm 
import cv2

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *
from spline import spline


class Visualizer:
    def __init__(self):
        
        rospy.init_node('visualizer')
        rospy.sleep(1)
        self.goals = []
        
        self.last_pos_path = rospy.get_time()
        
        self.truck = TruckModel()
        self.map_obj = Map()
        self.mapmodel = MapModel(self.map_obj)
        
        
        self.width = self.mapmodel.width * self.mapmodel.resolution
        self.height = self.mapmodel.height * self.mapmodel.resolution
        
        self.sim_reset_pub = rospy.Publisher('sim_reset', TruckState, queue_size=10)
        
        self.goal_pub = rospy.Publisher('truck_goals', cm.Path, queue_size=10)
        self.truck_pub = rospy.Publisher('truck_marker', Marker, queue_size=10)
        self.map_pub = rospy.Publisher("truck_map", OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher("truck_path", Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher("truck_ref_path", Path, queue_size=10)
        self.long_path_pub = rospy.Publisher("truck_long_path", Path, queue_size=10)
        self.trailer_path_pub = rospy.Publisher("truck_trailer_path", Path, queue_size=10)
        
        self.endpoint_pub = rospy.Publisher("end_point", PointStamped, queue_size=10)
        self.startpoint_pub = rospy.Publisher("start_point", PointStamped, queue_size=10)
        
        self.possible_path_pub = rospy.Publisher("possible_paths", Path, queue_size=10)
        self.visited_pub = rospy.Publisher("visited_points", PointStamped, queue_size=10)
        self.tovisit_pub = rospy.Publisher("tovisit_points", PointStamped, queue_size=10)
        
        self.setpoint_pub = rospy.Publisher("setpoints", PointStamped, queue_size=10)
        
        self.text_pub = rospy.Publisher("text_marker", Marker, queue_size=10)
        
        rospy.Subscriber('truck_state', TruckState, self.stateCallback)
        rospy.Subscriber('rviz_path', cm.Path, self.pathCallback)
        
        rospy.Subscriber('alg_startend',cm.Path, self.algStartEndCallback)
        
        
        rospy.Subscriber('possible_path',cm.Path, self.possiblePathCallback)
        
        rospy.Subscriber('to_visit_node',cm.Position, self.toVisitCallback)
        rospy.Subscriber('visited_node', cm.Position, self.visitedCallback)
        rospy.Subscriber('alg_startend',cm.Path, self.algStartEndCallback)
        
        rospy.Subscriber('long_path', cm.Path, self.longPathCallback)
        rospy.Subscriber('ref_path', cm.Path, self.refPathCallback)
        rospy.Subscriber('trailer_path', cm.Path, self.trailerPathCallback)
        
        
        rospy.Subscriber('clicked_point', PointStamped, self.pointClickedCallback)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goalPointCallback)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initPoseCallback)
        
        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)

        rospy.Subscriber('sim_text', String, self.textCallback)

        rospy.sleep(0.5)
        self.map_pub.publish(self.mapmodel.map)

    def textCallback(self, data):
        self.text = Marker()
        self.text.text = data.data
        self.text.header.frame_id = "truck"

        self.text.type = Marker.TEXT_VIEW_FACING
        self.text.action = Marker.ADD

        # 0 means that text is now immortal
        self.text.lifetime = rospy.Duration(0)
        
        self.text.scale.z = 220

        # sick blue color
        self.text.color.r = 0.1294117647
        self.text.color.g = 0.58823529411
        self.text.color.b = 0.95294117647
        self.text.color.a = 0.85
        
        self.text.pose.position.x = 0.0
        self.text.pose.position.y = 0.0
        self.text.pose.position.z = 110

        self.text_pub.publish(self.text)

        
    def possiblePathCallback(self, data):
        now = rospy.get_time()
        if now - self.last_pos_path > 5:
            dp = self.getDummyPointStamped()
           
            for _ in range(20):
                rospy.sleep(0.009)
                self.visited_pub.publish(dp)
            for _ in range(30):
                
                rospy.sleep(0.009)
                self.tovisit_pub.publish(dp)
                
        self.last_pos_path = now
        
        path = [(p.x*10, p.y*10) for p in data.path]
        p = TruckPath(path, 20)
        self.possible_path_pub.publish(p.path_msg)
        
    def visitedCallback(self, data):
        s = PointStamped()
        s.header.frame_id = "truck"
        s.point.x = data.x*10
        s.point.y = self.height - data.y*10
        self.visited_pub.publish(s)
    
    def toVisitCallback(self, data):
        s = PointStamped()
        s.header.frame_id = "truck"
        s.point.x = data.x*10
        s.point.y = self.height - data.y*10
        self.tovisit_pub.publish(s)
        
    def initPoseCallback(self, data):
        self.goals = []
        po = data.pose.pose
        q = po.orientation
        p = po.position
        
        ts = TruckState()
        ts.p = Position(p.x, self.height - p.y)
        e = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ts.theta1 = ts.theta2 = -e[2]
        self.sim_reset_pub.publish(ts)
        
        self.long_path_pub.publish(self.getDummyPath())
        self.ref_path_pub.publish(self.getDummyPath())
        self.path_pub.publish(self.getDummyPath())
        self.trailer_path_pub.publish(self.getDummyPath())
        self.endpoint_pub.publish(self.getDummyPointStamped())
        self.startpoint_pub.publish(self.getDummyPointStamped())
        
        dp = self.getDummyPointStamped()
        for _ in range(10):
            self.setpoint_pub.publish(dp)
            
        dp = self.getDummyPointStamped()
           
        dummypath = self.getDummyPath()
        for _ in range(20):
            rospy.sleep(0.009)
            self.visited_pub.publish(dp)
        for _ in range(30):
            
            rospy.sleep(0.009)
            self.tovisit_pub.publish(dp)
            
        for _ in range(1):
            
            rospy.sleep(0.009)
            self.possible_path_pub.publish(dummypath)
        
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

    def getDummyPoseStamped(self):
        ps = PoseStamped()
        ps.header.frame_id = 'truck'
        ps.pose.position.z = 5000
        return ps
    
    def getDummyPath(self):
        p = Path()
        p.header.frame_id = "truck"
        p.poses.append(self.getDummyPoseStamped())
        return p
        
    def getDummyPointStamped(self):
        p = PointStamped()
        p.header.frame_id = "truck"
        p.point.z = 5000
        return p

    def pointClickedCallback(self, data):
        p = data.point

        if self.goals == []:
            dp = self.getDummyPointStamped()
            for _ in range(10):
                self.setpoint_pub.publish(dp)

        self.goals.append((p.x, p.y))

        self.setpoint_pub.publish(data)

    def goalPointCallback(self, data):
        p = data.pose.position
        self.tp = []
        dp = self.getDummyPointStamped()

        dummypath = self.getDummyPath()
        for _ in range(20):
            rospy.sleep(0.009)
            self.visited_pub.publish(dp)
        for _ in range(30):
            rospy.sleep(0.009)
            self.tovisit_pub.publish(dp)

        for _ in range(1):
            rospy.sleep(0.009)
            self.possible_path_pub.publish(dummypath)

        if self.goals == []:
            dp = self.getDummyPointStamped()
            for _ in range(10):
                self.setpoint_pub.publish(dp)

        self.goals.append((p.x, p.y))

        gm = [cm.Position(x,self.height - y) for x,y in self.goals]
        self.goal_pub.publish(gm)

        ps = PointStamped()
        ps.header.frame_id = "truck"
        ps.point.x = p.x
        ps.point.y = p.y
        self.setpoint_pub.publish(ps)
        self.goals = []

    def stateCallback(self, msg):
        x = msg.p.x
        y = msg.p.y

        theta1 = msg.theta1
        theta2 = msg.theta2

        xf = x + self.truck.hl1 * math.cos(theta1)
        yf = y + self.truck.hl1 * math.sin(theta1)

        self.truck.setHeaderPosition(xf - self.truck.getTotalHeaderLength()* 0.5 * math.cos(theta1), \
                self.height - (yf- self.truck.getTotalHeaderLength() * 0.5 * math.sin(theta1)))
        self.truck.setHeaderDirection(-theta1)

        jx, jy = x - self.truck.hl2 * math.cos(theta1), y - self.truck.hl2 * math.sin(theta1)

        xtf = jx + self.truck.tl1 * math.cos(theta2)
        ytf = jy + self.truck.tl1 * math.sin(theta2)

        mtx, mty = xtf - 0.5*self.truck.getTotalTrailerLength() * math.cos(theta2), ytf - 0.5*self.truck.getTotalTrailerLength() * math.sin(theta2)

        self.truck.setTrailerPosition(mtx, self.height - mty)
        self.truck.setTrailerDirection(-theta2)

        self.truck_pub.publish(self.truck.header)
        self.truck_pub.publish(self.truck.trailer)

    def pathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]

        p = TruckPath(path, 30)
        self.path_pub.publish(p.path_msg)
        
        dp = self.getDummyPath()
        for _ in range(1):
            self.possible_path_pub.publish(dp)

    def refPathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]

        self.path_pub.publish(self.getDummyPath())
        self.trailer_path_pub.publish(self.getDummyPath())
        self.long_path_pub.publish(self.getDummyPath())

        p = TruckPath(path, 10)
        self.ref_path_pub.publish(p.path_msg)
        
    def longPathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]
        path = spline(path, 0, len(path)*7)
        dp = self.getDummyPointStamped()

        for _ in range(20):
            rospy.sleep(0.009)
            self.visited_pub.publish(dp)
        for _ in range(30):
            rospy.sleep(0.009)
            self.tovisit_pub.publish(dp)

        p = TruckPath(path, 20)
        self.long_path_pub.publish(p.path_msg)

    def trailerPathCallback(self, data):
        path = [(p.x, p.y) for p in data.path]
        path = spline(path, 0, len(path)*7)

        p = TruckPath(path, 30)
        self.trailer_path_pub.publish(p.path_msg)

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
        
        self.hl1 = 100
        self.hl2 = 210
        self.hl3 = 75
        
        self.tl1 = 75
        self.tl2 = 490
        self.tl3 = 135
        
        self.hw = 180
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
        self.header.scale.x = self.getTotalHeaderLength()
        self.header.scale.y = self.hw
        self.header.scale.z = 220

        # sick blue color
        self.header.color.r = 0.1294117647
        self.header.color.g = 0.58823529411
        self.header.color.b = 0.95294117647
        self.header.color.a = 0.85
        
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
        self.trailer.scale.x = self.getTotalTrailerLength()
        self.trailer.scale.y = self.tw
        self.trailer.scale.z = 220

        # sick red color
        self.trailer.color.r = 0.956862745
        self.trailer.color.g = 0.26274509803
        self.trailer.color.b = 0.21176470588
        self.trailer.color.a = 0.8
        
        #self.trailer.pose.position.x = 0.0
        #self.trailer.pose.position.y = -self.trailer.scale.x
        self.trailer.pose.position.z = 110
        
        #self.setHeaderDirection(math.pi/4+0.4)
        
    def getTotalHeaderLength(self):
        return self.hl1 + self.hl2 + self.hl3
    
    def getTotalTrailerLength(self):
        return self.tl1 + self.tl2 + self.tl3

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
    
