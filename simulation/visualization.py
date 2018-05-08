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
import virtual_blinker.msg as cm 
import cv2

from spline import spline



HEADER_FRAME = "virtualblinker"


class Visualizer:
    def __init__(self):
        
        
        rospy.init_node('visualizer')
        rospy.sleep(1)
        
        self.carmarker1 = CarMarker()
        self.carmarker2 = CarMarker()
        
        self.matrix = readImgToMatrix("map.png")
        self.scale = 1
        
        
        self.map, self.width, self.height = getOccupancyGrid(self.matrix, self.scale)
        
        self.path_pub = rospy.Publisher("rviz_car_path", Path, queue_size=10)
        self.path_pub2 = rospy.Publisher("rviz_car_path2", Path, queue_size=10)
        
        rospy.Subscriber('car_path', cm.Path, self.pathCallback)
        rospy.Subscriber('car_path2', cm.Path, self.pathCallback2)
        
        self.marker_pub1 = rospy.Publisher('rviz_car_marker', Marker, queue_size=10)
        self.marker_pub2 = rospy.Publisher('rviz_car_marker2', Marker, queue_size=10)
        self.map_pub = rospy.Publisher("rviz_map", OccupancyGrid, queue_size=10)
        
        rospy.Subscriber('car_state', cm.CarState, self.stateCallback)
        rospy.Subscriber('car_state2', cm.CarState, self.stateCallback2)

        rospy.sleep(0.5)
        self.map_pub.publish(self.map)

    
    def stateCallback(self, msg):
        x = msg.x #mm
        y = msg.y #mm
        
        x += 200/2
        y += 250/2
        
        x = x * 1000 #mm
        y = y * 1000 #mm
        
        x = x/250
        y = y/250
        
        
        theta = msg.theta
        
        xf = x- (self.carmarker1.length/2) *(math.cos(theta))
        yf = y - (self.carmarker1.length/2) * (math.sin(theta))

        self.carmarker1.setMarkerPosition(xf,yf)
        
        self.carmarker1.setMarkerDirection(theta)

        
        self.marker_pub1.publish(self.carmarker1.marker)
        
    def stateCallback2(self, msg):
        x = msg.x #mm
        y = msg.y #mm
        
        x += 200/2
        y += 250/2
        
        x = x * 1000 #mm
        y = y * 1000 #mm
        
        x = x/250
        y = y/250
        
        
        theta = msg.theta
        
        xf = x- (self.carmarker2.length/2) *(math.cos(theta))
        yf = y - (self.carmarker2.length/2) * (math.sin(theta))

        self.carmarker2.setMarkerPosition(xf,yf)
        
        self.carmarker2.setMarkerDirection(theta)

        
        self.marker_pub2.publish(self.carmarker2.marker)
        
    def pathCallback(self, data):
        p = getPath(data.path, 30)
        self.path_pub.publish(p)
        
        
        
    def pathCallback2(self, data):
        p = getPath(data.path, 30)
        self.path_pub2.publish(p)
        
    
    


class CarMarker:
    def __init__(self):
        
        self.length = 4000/250
        self.width  = 1800/250
        
        
        self.marker = Marker()
        self.marker.header.frame_id = HEADER_FRAME
        #self.marker.header.stamp = rospy.Time.now()

        # Set the namespace and id of the header, making it unique
        self.marker.ns = HEADER_FRAME
        self.marker.id = 1

        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        # 0 means that marker is now immortal
        self.marker.lifetime = rospy.Duration(0)
        
        # Truck measurments in mm
        self.marker.scale.x = self.length
        self.marker.scale.y = self.width
        self.marker.scale.z = 220

        # sick blue color
        self.marker.color.r = 0.1294117647
        self.marker.color.g = 0.58823529411
        self.marker.color.b = 0.95294117647
        self.marker.color.a = 0.85
        
        #self.marker.pose.position.x = 0.0
        #self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 110

        

    def setMarkerPosition(self, x, y):
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        
    
    # Takes angle in degrees and sets the direction of the marker
    def setMarkerDirection(self, angle):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]

    
# Takes a path (relative to current directory) to a an image file containing a Map representation
# (path='/map.png' for file 'map.png, located in current directory)
# The file should be in '.png'-format
#
# Returns an array of rows, where each row is an array of elements
def readImgToMatrix(path):
    dirpath = dirname(abspath(__file__))
    matrix = np.asarray(cv2.imread(os.path.join(dirpath, path), 0), dtype=np.uint8).tolist()
    

    return matrix
        
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
                oc.data.append(20)
            else:
                oc.data.append(0)
                
            
    return oc, width * scale, height * scale


def getPath(dp, z, splined=False, scaled=False):
    
    path = [((p.x + 100 )/0.250, (p.y + 125) / 0.250) for p in dp]
    
    if splined:
        path = spline(path)
    
    path_msg = Path()
    path_msg.header.frame_id = HEADER_FRAME

    for x,y in path:
        ps = PoseStamped()

        ps.header.frame_id = HEADER_FRAME

        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z

        ps.pose.orientation.w = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0

        path_msg.poses.append(ps)
    return path_msg

if __name__=="__main__":
    Visualizer()
    rospy.spin()
    
