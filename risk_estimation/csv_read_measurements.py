"""
this is a tool to retrace rosbag csv files measurements to step throught the particle
filter as it happens. (done offline)


"""

import math
import driver
import csv
from collections import OrderedDict
import Intersection
import numpy as np
#from pycallgraph import PyCallGraph
#from pycallgraph.output import GraphvizOutput
import config

my_risk_estimator = None
csv_file = config.GENERAL_OPTIONS['input-csv-file'] 

n_particles = 400
myintersection = None

def hardcode_intersection():
    global my_risk_estimator
    global myintersection

    alignment = 'east-west'
    intersectiontype = Intersection.IntersectionType.GIVE_WAY_4
    center = (0.,0.)
    lane_width = 7.5
    myintersection = Intersection.Intersection(center,lane_width,intersectiontype, alignment)
    #my_inital_measurements = OrderedDict()
    #my_inital_measurements[0] = (-60.,-3.25,0,50/3.6)
    #my_inital_measurements[1] = (-3.25,60.,3*math.pi/2,50/3.6)

    #my_risk_estimator = driver.risk_estimator(n_particles,interval,myintersection,my_inital_measurements,pose_cov,speed_deviation,[1,2])

def readcsv():
    global csv_file
    global my_risk_estimator
    global n_particles

    pose_cov = config.PARAMETERS['pose-covariance']
    speed_deviation = config.PARAMETERS['speed-deviation']

    with open(csv_file, 'r') as csvfile:
        csvreader = csv.reader(csvfile,delimiter=',')
        c = 1
        for row in csvreader:
            if c == 1:
                c = c + 1
                continue
            t = float(row[1])
            v1_measurements = (float(row[2]), float(row[3]), float(row[4]),  float(row[5] ))
            v2_measurements = (float(row[8]), float(row[9]), float(row[10]), float(row[11]))

            #contstruct ordered dict
            measurement_vector = OrderedDict()
            measurement_vector[0] = v1_measurements 
            measurement_vector[1] = v2_measurements 
            if t < 0.001:
                my_risk_estimator = driver.RiskEstimator(n_particles,0.1,myintersection,measurement_vector,pose_cov,speed_deviation,[1,2])
                c = c + 1
                continue
            

            my_risk_estimator.update_state(t,measurement_vector)
            print("t = " + str(t) + str(my_risk_estimator.get_risk()))
            c = c + 1




def main():
    #graphviz = GraphvizOutput()
    #graphviz.output_file = './perf/basic.png'

    hardcode_intersection()
    # with PyCallGraph(output=graphviz):
    readcsv()
    #measurement = get_measurement()

main()