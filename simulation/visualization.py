#!/usr/bin/env python
import sys
sys.path.append("..")

from utils.spline import *
import rospy
import math
import tf
import os
from threading import Thread
from os.path import dirname, abspath
import numpy as np
from std_msgs.msg import Int8, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
import virtual_blinker.msg as cm 
import cv2


colors = [
    (230, 25, 75),
    (60, 180, 75),
    (0, 130, 200),
    (245, 130, 48),
    (145, 30, 180),
    (230, 25, 75),
    (60, 180, 75),
    (0, 130, 200),
    (245, 130, 48),
    (145, 30, 180)


]

HEADER_FRAME = "virtualblinker"


class Visualizer:
    def __init__(self):
        

        rospy.init_node('visualizer')

        
        nr_cars = 10

        self.markers = [CarMarker(colors[i]) for i in range(nr_cars)]

        
        matrix = readImgToMatrix("map.png")
        _map = getOccupanceGrid(matrix)
        
        
        self.marker_pubs = []
        self.path_pubs = []
        for i in range(nr_cars):

            rospy.Subscriber('car_path' + str(i), cm.Path, self.pathCallback)
            rospy.Subscriber('true_car_state' + str(i), cm.CarStateTrue, self.stateCallback)
            self.marker_pubs.append(rospy.Publisher('rviz_car_marker' + str(i), Marker, queue_size=10))
            self.path_pubs.append(rospy.Publisher("rviz_car_path" + str(i), Path, queue_size=10))


        map_pub = rospy.Publisher("rviz_map", OccupancyGrid, queue_size=10)

        #sleep to let rviz start before publishing. Also note: you have to wait a little while after registering a publisher after you can use it,
        rospy.sleep(1.5)
        
        map_pub.publish(_map)

    
    def stateCallback(self, msg):
        x = msg.x 
        y = msg.y 
        

        # sim coordinate system has origo in (100,125), and rviz has origo in corner
        x += 100
        y += 125

        #4 pixels per meter. size of map is 800 x 1000 and sim coordinate system is 200*250
        x *= 4
        y *= 4
        
        theta = msg.theta

        m = self.markers[msg.id-1]
        
        #x,y is front of car. find centre of car
        xc = x- (m.length/2) *(math.cos(theta))
        yc = y - (m.length/2) * (math.sin(theta))

        m.setMarkerPosition(xc,yc)
        m.setMarkerDirection(theta)

        
        self.marker_pubs[msg.id-1].publish(m.marker)
        
    
        
    def pathCallback(self, data):
        p = getPathMessage(data.path)
        self.path_pubs[data.id-1].publish(p)
        
        
    
class CarMarker:
    def __init__(self, (r,g,b)):
        
        #car is 4x1.8 meters, meters / pixel = 4
        self.length = 4*4
        self.width  = 1.8*4
        
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
        self.marker.color.r = r/255.0
        self.marker.color.g = g/255.0
        self.marker.color.b = b/255.0
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

    
#read map image and return matrix with color values
def readImgToMatrix(path):
    dirpath = dirname(abspath(__file__))
    matrix = np.asarray(cv2.imread(os.path.join(dirpath, path), 0), dtype=np.uint8).tolist()

    return matrix
        
#transform matrix to an OccupancyGrid-message
def getOccupanceGrid(matrix):
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
    
    oc.info = MapMetaData(load_time, 1, width, height, origin)
    
    for row in range(height):
        for col in range(width):
            c = matrix[row][col]
            
            if c == 0:
                oc.data.append(20)
            else:
                oc.data.append(0)
                
            
    return oc


def getPathMessage(path):
    
    path = [((p.x + 100 )*4, (p.y + 125)*4) for p in path]
    
    path_msg = Path()
    path_msg.header.frame_id = HEADER_FRAME

    for x,y in path:
        ps = PoseStamped()

        ps.header.frame_id = HEADER_FRAME

        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 30

        ps.pose.orientation.w = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0

        path_msg.poses.append(ps)
    return path_msg

if __name__=="__main__":
    Visualizer()
    rospy.spin()
    
