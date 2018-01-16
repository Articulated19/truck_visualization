#!/usr/bin/env python
"""
Copyright (c) 2017, Lars Niklasson
Copyright (c) 2017, Filip Slottner Seholm
Copyright (c) 2017, Fanny Sandblom
Copyright (c) 2017, Kevin Hoogendijk
Copyright (c) 2017, Nils Andren
Copyright (c) 2017, Alicia Gil Martin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
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



HEADER_FRAME = "truck"


class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer')
        rospy.sleep(1)
        self.goals = []
        
        self.last_pos_path = rospy.get_time()
        
        self.truck = TruckModel()
        self.map_obj = Map(centerline=True)
        matrix, self.scale = self.map_obj.getMapAndScale()
        
        self.map, self.width, self.height = getOccupancyGrid(matrix, self.scale)
        
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
        self.map_pub.publish(self.map)

    
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
    
    def textCallback(self, data):
        self.text = Marker()
        self.text.text = data.data
        self.text.header.frame_id = HEADER_FRAME

        self.text.type = Marker.TEXT_VIEW_FACING
        self.text.action = Marker.ADD

        # 0 means that text is now immortal
        self.text.lifetime = rospy.Duration(0)
        
        self.text.scale.z = 220

        # sick blue color
        # self.text.color.r = 0.1294117647
        # self.text.color.g = 0.58823529411
        # self.text.color.b = 0.95294117647
        self.text.color.r = 1
        self.text.color.g = 1
        self.text.color.b = 1
        self.text.color.a = 1
        
        self.text.pose.position.x = 0.0
        self.text.pose.position.y = 0.0
        self.text.pose.position.z = 110

        self.text_pub.publish(self.text)
    
    def removeVisited(self):
        dp = self.getDummyPointStamped()
       
        for _ in range(20):
            rospy.sleep(0.009)
            self.visited_pub.publish(dp)
        for _ in range(30):
            
            rospy.sleep(0.009)
            self.tovisit_pub.publish(dp)
        
    def possiblePathCallback(self, data):
        now = rospy.get_time()
        if now - self.last_pos_path > 5:
            self.removeVisited()
                
        self.last_pos_path = now
        
        
        self.possible_path_pub.publish(self.getPath(data.path, 22, scaled=True))
        
    def getPointStamped(self, x, y):
        s = PointStamped()
        s.header.frame_id = HEADER_FRAME
        s.point.x = x * self.scale
        s.point.y = self.height - y * self.scale
        return s
        
    def visitedCallback(self, data):
        s = self.getPointStamped(data.x, data.y)
        self.visited_pub.publish(s)
    
    def toVisitCallback(self, data):
        s = self.getPointStamped(data.x, data.y)
        self.tovisit_pub.publish(s)
        
    def removeSetPoints(self):
        dp = self.getDummyPointStamped()
        for _ in range(10):
            self.setpoint_pub.publish(dp)
            
        
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
        self.possible_path_pub.publish(self.getDummyPath())
        self.removeSetPoints()
        self.removeVisited()
        
        
    def algStartEndCallback(self, data):
        p = data.path
        
        s = self.getPointStamped(p[0].x, p[0].y)
        self.startpoint_pub.publish(s)
        
        e = self.getPointStamped(p[1].x, p[1].y)
        self.endpoint_pub.publish(e)

    def mapUpdateHandler(self, data):
        obst = data.data
        add = self.map_obj.addObstacle(obst)
        if not add:
            rem = self.map_obj.removeObstacle(obst)
            if not rem:
                print "can't add or remove obstacle"
                
        m, s = self.map_obj.getMapAndScale()
        oc, _ , _ = getOccupancyGrid(m, s)
        self.map_pub.publish(oc)

    def getDummyPoseStamped(self):
        ps = PoseStamped()
        ps.header.frame_id = HEADER_FRAME
        ps.pose.position.z = 5000
        return ps
    
    def getDummyPath(self):
        p = Path()
        p.header.frame_id = HEADER_FRAME
        p.poses.append(self.getDummyPoseStamped())
        return p
        
    def getDummyPointStamped(self):
        p = PointStamped()
        p.header.frame_id = HEADER_FRAME
        p.point.z = 5000
        return p

    def pointClickedCallback(self, data):
        p = data.point

        if self.goals == []:
            self.removeSetPoints()

        self.goals.append((p.x, p.y))

        self.setpoint_pub.publish(data)

    def goalPointCallback(self, data):
        p = data.pose.position
        self.tp = []
        
        self.removeVisited()
        self.possible_path_pub.publish(self.getDummyPath())
        if self.goals == []:
            self.removeSetPoints()

        self.goals.append((p.x, p.y))

        gm = [cm.Position(x,self.height - y) for x,y in self.goals]
        self.goal_pub.publish(gm)

        ps = PointStamped()
        ps.header.frame_id = HEADER_FRAME
        ps.point.x = p.x
        ps.point.y = p.y
        self.setpoint_pub.publish(ps)
        self.goals = []

    

    def pathCallback(self, data):
        p = self.getPath(data.path, 30)
        self.path_pub.publish(p)
        
        self.possible_path_pub.publish(self.getDummyPath())

    def refPathCallback(self, data):
        
        p = self.getPath(data.path, 10)
        self.ref_path_pub.publish(p)
        
    def longPathCallback(self, data):
        
        self.removeVisited()
        p = self.getPath(data.path, 20, splined=True)
        self.long_path_pub.publish(p)

    def trailerPathCallback(self, data):
        p = self.getPath(data.path, 25, splined=True)
        
        self.trailer_path_pub.publish(p)
        
    def getPath(self, dp, z, splined=False, scaled=False):
        scale = 1
        if scaled:
            scale = self.scale
        
        path = [(p.x * scale, p.y * scale) for p in dp]
        
        if splined:
            path = spline(path, 0, len(path)*7)
        
        path_msg = Path()
        path_msg.header.frame_id = HEADER_FRAME

        for x,y in path:
            ps = PoseStamped()

            ps.header.frame_id = HEADER_FRAME

            ps.pose.position.x = x
            ps.pose.position.y = self.height - y
            ps.pose.position.z = z

            ps.pose.orientation.w = 0.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0

            path_msg.poses.append(ps)
        return path_msg


class TruckModel:
    def __init__(self):
        
        self.hl1 = 95
        self.hl2 = 220
        self.hl3 = 110
        
        self.tl1 = 75
        self.tl2 = 445+102.5+85
        self.tl3 = 0
        
        self.hw = 180
        self.tw = 180
        
        # Create the header marker, and set a static frame
        self.header = Marker()
        self.header.header.frame_id = HEADER_FRAME
        #self.header.header.stamp = rospy.Time.now()

        # Set the namespace and id of the header, making it unique
        self.header.ns = HEADER_FRAME
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

    
        
        
def getOccupancyGrid(matrix, scale):
    stamp = rospy.Time.now()

    oc = OccupancyGrid()
    oc.header.frame_id = HEADER_FRAME
    oc.header.stamp = stamp

    # The time at which the map was loaded
    load_time = stamp
    
    # The origin of the map [m, m, rad].  This is the real-world pose of the
    # cell (0,0) in the map.
    origin = Pose(Point(0,0,0), Quaternion(0,0,0,0))

    matrix = matrix[::-1]
    
    width = len(matrix[0])
    height = len(matrix)
    
    oc.info = MapMetaData(load_time, scale, width, height, origin)
    
    for row in range(height):
        for col in range(width):
            c = matrix[row][col]
            if c == 0:
                oc.data.append(100)
            elif c == 1:
                oc.data.append(0)
                
            else:
                oc.data.append(50)
    return oc, width * scale, height * scale


if __name__=="__main__":
    Visualizer()
    rospy.spin()
    
