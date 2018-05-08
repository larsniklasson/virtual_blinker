#test the whole risk estimation
import unittest
import sys
sys.path.append("..")
from Intersection import Intersection
from Intersection import IntersectionType
from driver import risk_estimator
from collections import OrderedDict
import utils
import driver
import P_estimate

class RiskEstimationTest(unittest.TestCase):
    
    def setUp(self):
        self.center_point = (0,0) # change as needed
        self.lane_width = 3.5
        self.intersection_gw_ns = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'north-south')
        self.intersection_gw_ew = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'east-west')
    
    def test_initialization(self):

        #test with just two vehicles on the eastwest aligned one
        v0 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'east',9)
        v1 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',15)
        v0s = 5
        v1s = 7
        n_particles = 400
        interval = 0.1 #in seconds

        initial_measurements = OrderedDict({ 0: (v0[0],v0[1],v0[2],v0s), 1: (v1[0],v1[1],v1[2],v1s)})
        #print(initial_measurements)
        estimator = risk_estimator(n_particles,interval,self.intersection_gw_ew,initial_measurements)
        v0 = P_estimate.vehicle_project_foward(v0,v0s,interval)
        v1 = P_estimate.vehicle_project_foward(v1,v1s,interval)

        print(estimator.get_risk())
        second_measurement = OrderedDict({ 0: (v0[0],v0[1],v0[2],v0s), 1: (v1[0],v1[1],v1[2],v1s)})
        estimator.update_state(1,second_measurement)

        v0 = P_estimate.vehicle_project_foward(v0,v0s,interval)
        v1 = P_estimate.vehicle_project_foward(v1,v1s,interval)

        third_measurement = OrderedDict({ 0: (v0[0],v0[1],v0[2],v0s), 1: (v1[0],v1[1],v1[2],v1s)})
        #estimator.update_state(2,third_measurement)

        pass