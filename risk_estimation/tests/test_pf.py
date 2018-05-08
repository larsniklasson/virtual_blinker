import sys
sys.path.append("..")
import unittest
#test the particle filter
import utils
import particle_filter
import Intersection
import numpy as np

class ParticleFilterTest(unittest.TestCase):
    def setUp(self):
        self.center_point = (0,0) # change as needed
        self.lane_width = 3.5
        self.intersection_gw_ns = Intersection.Intersection(self.center_point,3.5,Intersection.IntersectionType.GIVE_WAY_4,'north-south')
        self.intersection_gw_ew = Intersection.Intersection(self.center_point,3.5,Intersection.IntersectionType.GIVE_WAY_4,'east-west')
    
    def test_initial_particle_generation(self):
        #generate 200 particles:
        v0 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'east',9)
        v1 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',15)

        v0s = 27
        v1s = 17

        v0_state = particle_filter.state_vector(1,1,1,v0,v0s)
        v1_state = particle_filter.state_vector(1,1,1,v1,v1s)

        v0particles = utils.generate_inital_particles(v0_state,200)
        #v0pf = particle_filter.particle_filter('east',self.intersection_gw_ew,200,(v0[0],v0[1],v0[2],v0s),0.1)
        v0poses  = np.array([x.P for x in v0particles])
        print("v0 actual pose is " + str(v0))
        #print(str(np.array(v0poses)))
        print(v0poses.shape)
        print("average is " + str(np.mean(v0poses,axis=0)))
