import unittest
import sys
import math
sys.path.append("..")
import E_estimate
import Intersection
#from Intersection import Intersection
#from Intersection import IntersectionType
#from Intersection import IntersectionCourses
import utils
import numpy as np
import plotter
import P_estimate

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import sys
import subprocess
"""

make an intersection
put a car in south
test exemplar parth on different courses at different times,
plot them
"""

class ExemplarPathTest(unittest.TestCase):
    def setUp(self):
        self.center_point = (0,0) # change as needed
        self.lane_width = 7.5
        self.intersection_gw_ns = Intersection.Intersection(self.center_point,7.5,Intersection.IntersectionType.GIVE_WAY_4,'north-south')
        self.intersection_gw_ew = Intersection.Intersection(self.center_point,7.5,Intersection.IntersectionType.GIVE_WAY_4,'east-west')
    
    def test_southvehicle_paths(self):
        fig , ax = plt.subplots(figsize=(10,10))
        ax.set_title("exemplar path")
        ax.spines['left'].set_position('center')
        ax.spines['bottom'].set_position('center')
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.autoscale(False)
        #plt.xticks(np.arange(min(-10), max(10)+1, 0.5))
        #plt.yticks(np.arange(min(-10), ma1(10)+1, 0.5))
        ax.add_patch(
            patches.Rectangle(
                (-7.5,-7.5),  #origin
                7.5*2,   #width
                7.5*2,   #height
                fill= False
            )
        )
        #v0 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',9)
        v0 = (3.5,-7.3,math.pi/2) #south
        v1 = (7.4,3.5,math.pi) #east
        v2 = (-3.5,7.4,3*math.pi/2) #north
        v3 = (-7.4,-3.5,0) #west
        speed = 13
        course = 3
        interval = 0.09
        pose_covariance = np.eye(3)*0.01
        
        for i in range(20):
            time_after = (i+1)*interval
            print("time after = " + str(time_after))
            p0 = P_estimate.P_estimate(v3,speed,course,self.intersection_gw_ew,time_after,'west',pose_covariance)
            p1 = P_estimate.project_on_exemplarpath(v3 ,speed,course,self.intersection_gw_ew,time_after,'west')

            plot_vehicle(fig,ax,p0,time_after)
            plot_vehicle(fig,ax,p1,time_after)
        
        filename = "./plots/plot_" +str(int((round(time.time()*1000))))
        fig.savefig(filename)
        plt.close(fig)
        
        pass

def plot_vehicle(fig,ax, pose,t):
    #ax.set_title("t = "+str(t)  + "\n" +str(riskdict) + "\n")
    plotter.draw_arrow(fig,ax,pose,1)

    
    
    