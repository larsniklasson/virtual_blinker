import unittest
import sys
import math
sys.path.append("..")
import E_estimate
#import Intersection
from Intersection import Intersection
from Intersection import IntersectionType
from Intersection import IntersectionCourses
import utils
import numpy as np

class Es_estimate_give_way_test(unittest.TestCase):
    """
    Testing of Es_estimate method using a simple give way intersection:
    Possible code paths and conditions:

    1) whether you have the right of way or not
    2) whether you are inside the intersection four corners or not
    3) test case when who does not have the right of way continues without slowing down
    4) whether intersection is aligned northsouth or east west
    5) if inside intersection, which course {0,1,2,3} are you taking? and where are you coming from?

    special case: in a give way, you are waiting at the intersection for a timegap
    """


    def setUp(self):
        self.center_point = (-15,7) # a random point
        self.lane_width = 3.5
        self.intersection_gw_ns = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'north-south')
        self.intersection_gw_ew = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'east-west')

    def test_inside_intersection_has_right_of_way(self):
        #the vehicle in question is inside intersection, vehicle entered the intersection
        #from a lane which has the right of way.
        #in this case, expectation is to go
        v0 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'east',9)
        v1 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',15)
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'west',7)
        v3 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',12)

        v0s  = 9. #v4 time to reach is (12-3.5)/9 = 0.9444s later   absolute time = 13.9444
        v1s  = 5  #v1 time to reach is (9-3.5)/5 =  1.1s later, absolute time = 14.1
        v2s  = 8. #v2 time to reach is (15-3.5)/8 = 1.4375s later, absoute time = 14.4375
        v3s  = 10

        cord = utils.offset_relative(self.intersection_gw_ew.center,(0.5,-1.))
        theta = math.radians(90+30)
        v4 = (cord[0],cord[1],theta)
        v4s = 10

        courses = [IntersectionCourses.LEFT_TURN]*5 
        speeds = [v0s,v1s,v2s,v3s,v4s]
        ved = ['east','north','west','south','south']
        poses = [v0,v1,v2,v3,v4]
        t = 13
        self.assertEqual(E_estimate.Es_estimate(v4,v4s,Intersection.IntersectionCourses.LEFT_TURN,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (0,1))

    def test_outside_intersection_no_right_of_way(self):
        """
        In here, vehicles do not have right of way with respect to other vehicles
        test cases:
        1) vehicle in question intends to make a left turn but will not have enough time to make the maneuver
           until vehicle who has the right of way comes at the intersection 
        2) vehicle in question intends to make a left turn and will have more than enough time to make the 
           maneuver although it does not have the right of way. in this case, I can still make the maneuver
           because there is enough time
        3) vehicle in question intends to make a left turn and the time gap in here will be just slightly 
           less than adequate. probablity of stopping (s = 0) will be somewhere around 0.1 and going (s = 1) will be 0.9
           because it still has time but not so sure
        4) vehicle in question intends to make a left turn and the time gap in here will be just slightly 
           on the 'not enough' side. probablity of stopping (s = 0) will be somewhere around 0.9 and going (s = 1) will be 0.1
           because expectation to stop is higher due to small time gap

        The intended courses for all these cases are the same because the probablity density calculation is
        done exactly same for other causes, for now.
        """

        #case when intersection is east-west aligned
        #vehicles, v1 and v3 has no right of way
        v1 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'east',9)
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',15)
        v3 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'west',7)
        v4 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',12)
        poses = [v1,v2,v3,v4]

        courses = [IntersectionCourses.LEFT_TURN]*4 
        t = 13
        ved = ['east','north','west','south']

        #fine tune v1 and v3 so that it can do testing for 4 cases above:
        #current time is 13
        #speeds are in meters per second

        v1s  = 5  #v1 time to reach is (9-3.5)/5 =  1.1s later, absolute time = 14.1
        v2s  = 8. #v2 time to reach is (15-3.5)/8 = 1.4375s later, absoute time = 14.4375
        v3s  = 10
        v4s  = 9. #v4 time to reach is (12-3.5)/9 = 0.9444s later   absolute time = 13.9444
        speeds = [v1s,v2s,v3s,v4s]

        #by the time v1 reaches intersection, v4 would be already navigating the intersection.
        #no need to worry about v4,
        #smallest positive time gap would be v2, which is 0.3375s
        #v1 has 0.3375seconds to execute maneuver before v2 comes at the intersection
        #since this gap is less than most riskiest but safe maneuver, expectation is to stop:

        self.assertEqual(E_estimate.Es_estimate(v1,v1s,ved[0],self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (1,0))
        
        #but suppose speed of v1 is faster. this would imply more time gap available.
        #this can also imply that v1 reaches before v4 if speed is increased sufficiently
        #lets see what happens
        
        v1s  = 6  #v1 time to reach is (9-3.5)/6 =  abs time 13.91666
        speeds = [v1s,v2s,v3s,v4s]
        #in this case, v1 is able to reach intersection faster than v4 even, with
        #time gap 0.02777844

        self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (1,0))

        v1s  = 7  #v1 time to reach is (9-3.5)/7 =  abs time 13.78
        speeds = [v1s,v2s,v3s,v4s]
        #in this case, v1 is able to reach intersection faster than v4 even, with
        #time gap 0.15873015873 second, not enough
        self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (1,0))

        #it seems that the time gap we gain with this distance is not enough at all

        #lets change v2 and v4 so it is a bit further away:
        
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',20)
        v4 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',26)
        v1s  = 10  #v1 time to reach is (9-3.5)/10, abstime = 13.55
        poses = [v1,v2,v3,v4]
        speeds = [v1s,v2s,v3s,v4s]
        #in this case, v1 is able to reach intersection faster than v4 even, with
        #time gap 1.5125 second, not enough
        self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (1,0))


        #increase vehicle speed even further:
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',30)
        v4 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',36)
        v1s  = 20  #v1 time to reach is (9-3.5)/20, abstime = 13.275
        poses = [v1,v2,v3,v4]
        speeds = [v1s,v2s,v3s,v4s]
        #in this case, v1 is able to reach intersection faster than v4 even, with
        #time gap  3.0375
        #self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (0.997461,1.-0.997461))
        np.testing.assert_almost_equal(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (0.9974614,1.-0.9974614))

        #increase vehicle speed even further :
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',40)
        v2s = 4
        v4 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',46)
        v4s = 3
        v1s  = 2  #v1 time to reach is (9-3.5)/20, abstime = 13.275
        poses = [v1,v2,v3,v4]
        speeds = [v1s,v2s,v3s,v4s]
        #in this case, v1 is able to reach intersection faster than v4 even, with
        #time gap  3.0375
        #self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (0.997461,1.-0.997461))
        #np.testing.assert_almost_equal(E_estimate.Es_estimate(0,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved),  (0.9974614,1.-0.9974614))

    def test_outside_intersection_have_right_of_way(self):
        """
        In here, Intersection is vehicles travelling towards intersection from each of, and other 
        vehicles scattered around in random points but towards the intersection
        only vehicles that have the right of way are tested in this testcase
        """

        #case when intersection is east-west aligned
        #vehicles, v2 and v4 has right of way
        v1 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'east',9)
        v2 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'north',15)
        v3 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'west',7)
        v4 = utils.place_vehicle(self.intersection_gw_ew.center,self.lane_width,'south',12)

        #when vehicles are outside intersection, Es_ estimation does not depend on intended course, when they have
        #right of way
        courses = [1]*4
        speeds = [15]*4
        poses = [v1,v2,v3,v4]
        t = 13
        ved = ['east','north','west','south']

        #these vehicles have the right of way, so s=0 is 0 and s=1 is 1, (0,1)
        self.assertEqual(E_estimate.Es_estimate(1,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved), \
                        (0,1))
        self.assertEqual(E_estimate.Es_estimate(3,self.intersection_gw_ew,courses,poses,speeds,t,0.1,ved), \
                        (0,1))

        #case when intersection is north-south aligned
        #v1,v2,v3,v4 have same pose and speed, but tested against north-south intersection:
        #these vehicles have the right of way, so s=0 is 0 and s=1 is 1, (0,1)
        self.assertEqual(E_estimate.Es_estimate(0,self.intersection_gw_ns,courses,poses,speeds,t,0.1,ved), \
                        (0,1))
        self.assertEqual(E_estimate.Es_estimate(2,self.intersection_gw_ns,courses,poses,speeds,t,0.1,ved), \
                        (0,1))
        

class Es_estimate_primtive_test(unittest.TestCase):
    def setUp(self):
        self.center_point = (0,0) # change as needed
        self.lane_width = 3.5
        self.my_intersection = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'north-south')


#Es_estimate (s, n , intersection_info, courses, poses,speeds ,t,interval):
#def project_vehicle(pose, speed, intersection_info,interval): # todo: need to vectorize this (give multiple poses,speed and returns time in np array)
    def test_vehicle_2p_intersect(self):
        #this is a random point, pose and speed I took for test
        p1 = (-5.0,-4.0)
        p2 = (2.0,-7.0)
        pose = (15.0,6.0,math.radians(180+60))
        speed = 18.0
        #by manual calculation, intersection point is (6.4045925, -8.8876825)
        #distance from where pose, is to the intersection point is:
        #
        #time to reach that point is : 
        #17.190814998477023 / 18 =  1.6092388677305383 seconds
        self.assertAlmostEqual(E_estimate.vehicle_2p_intersect(pose,speed,p1,p2),0.9550452776931679)

        #when lines are parallel,
        #the testing method should return False:
        #this method has a caveat that when the pose lies directly in line with
        #p1 and p2 ( paralllel and overlapping )
        #it will return False. todo: have to fixthis
        p1 = (0.,5.)
        p2 = (0.,9.)
        pose = (5.,0.,math.radians(90))
        #pose = (0.,0.,math.radians(90))
        speed = 7.0
        #print(E_estimate.vehicle_2p_intersect(pose,speed,p1,p2))
        self.assertEqual(E_estimate.vehicle_2p_intersect(pose,speed,p1,p2),False)


      #vehicle_2p_intersect(pose, speed, p1, p2):

    def test_time_toreach(self):
        v_pose = (0,0,-math.pi)
        speed = 2 #meter per second
        p0 = (-60,0)
        self.assertEqual(E_estimate.time_toreach(v_pose,speed,p0),30)
    
    def test_foward_project_vehicle(self):
        #for foward projection, we dont actually require courses or vehicle_entering_direction parameter
        #because we already know the directions its coming from using the cordinates
        #so, for this, courses and vehicle_entering_direction parameters are empty lists
        v1 = (-7,-2,0)
        v2 = (-2,7,math.radians(180+90))
        v3 = (8,2,math.radians(180))
        v4 = (2,-8,math.radians(90))
        speed = 4 
        #print(E_estimate.project_vehicle(v1,speed,self.my_intersection,24))
        interval = 24 # this is not used yet.
        courses = []
        ved = []
        self.assertAlmostEqual(E_estimate.project_vehicle(v1,speed,self.my_intersection,interval, courses,ved),0.875)
        self.assertAlmostEqual(E_estimate.project_vehicle(v2,speed,self.my_intersection,interval, courses,ved),0.875)
        self.assertAlmostEqual(E_estimate.project_vehicle(v3,speed,self.my_intersection,interval, courses,ved),1.125)
        self.assertAlmostEqual(E_estimate.project_vehicle(v4,speed,self.my_intersection,interval, courses,ved),1.125)

    def test_backward_project_vehicle(self):
        #we have four courses and four directions, so we have 16 different individual tests.
        #each testing would involve 4 vehicles, 2 on the exact arc/path that are to be taken in the ideal course
        # and two from both sides of the arc/path
        #print("testing backward projection..")

        self.left_turn_test()
        self.straight_ahead_test()
        self.right_turn_test()
        self.u_turn_test()
    
    def u_turn_test(self):
        speed = 7
        interval = 3

        #in here i want to make one more vehicle that lies in the midpoint of the arc,
        #that would make 90 degrees from base
        #u-turn , coming from south,
        #point directly on arc:
        p1 = (0.875 , 1.515544457 - 3.5) # rect(3.5/2, 60degree)
        p2 = (-1.077407582, 1.379018819 - 3.5) # rect(3.5/2, 128degree)
        #point left of arc:
        p3 = (0.2811888179, 1.217962581 - 3.5) # rect(2.5/2, 77degree)
        #point right of arc:
        p4 = (-2.05547728 , 0.9151574469 - 3.5) # rect(4.5/2, 156degree)
        p5 = (0 , 1.75 - 3.5) # rect(3.5/2, 90degree) #midpoint
        p1time = -1 *((math.radians(60)*(3.5/2)) / speed)
        p2time = -1 *((math.radians(128)*(3.5/2)) / speed)

        p3time = -1 *((math.radians(77)*(2.5/2)) / speed)
        p4time = -1 *((math.radians(156)*(4.5/2)) / speed)
        p5time = -1 *((math.radians(90)*(3.5/2)) / speed)

        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"south"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"south"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"south"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"south"),p4time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p5,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"south"),p5time)

        #case when vehicle is coming from east and doing u turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        p5 = utils.rotate_point(self.my_intersection.center,p5,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"east"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"east"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"east"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"east"),p4time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p5,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"east"),p5time)

        #case when vehicle is coming from north and doing u turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        p5 = utils.rotate_point(self.my_intersection.center,p5,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"north"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"north"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"north"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"north"),p4time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p5,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"north"),p5time)

        #case when vehicle is coming from west and doing u turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        p5 = utils.rotate_point(self.my_intersection.center,p5,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"west"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"west"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"west"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"west"),p4time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p5,speed,self.my_intersection,interval,IntersectionCourses.U_TURN,"west"),p5time)


    def right_turn_test(self):
        speed = 7
        interval = 3

        #right turn , coming from south,
        #point directly on arc:
        p1 = (-1.320741765 +3.5 , 1.148103301 - 3.5) # rect(3.5/2, 139degree)
        p2 = (-0.30388431 +3.5 , 1.723413568 - 3.5) # rect(3.5/2, 100degree)
        #point left of arc:
        p3 = (-1.125 +3.5 , 1.948557159 - 3.5) # rect(4.5/2, 120degree)
        #point right of arc:
        p4 = (-0.508420803 +3.5 , 1.141931822 - 3.5) # rect(2.5/2, 114degree)
        p1time = -1 *((math.radians(180-139)*(3.5/2)) / speed)
        p2time = -1 *((math.radians(180-100)*(3.5/2)) / speed)
        p3time = -1 *((math.radians(180-120)*(4.5/2)) / speed)
        p4time = -1 *((math.radians(180-114)*(2.5/2)) / speed)
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"south"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"south"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"south"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"south"),p4time)

        #case when vehicle is coming from east and doing right turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"east"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"east"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"east"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"east"),p4time)

        #case when vehicle is coming from north and doing right turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"north"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"north"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"north"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"north"),p4time)

        #case when vehicle is coming from west and doing right turn
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"west"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"west"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"west"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.RIGHT_TURN,"west"),p4time)

    def straight_ahead_test(self):
        speed = 3.19
        interval = 3

        #case for straight ahead, coming from south
        #directly on path:
        p1 = (self.my_intersection.corner_bottom_right[0] - 3.5/2, self.my_intersection.corner_bottom_right[1] + 4)
        p2 = (self.my_intersection.corner_bottom_right[0] - 3.5/2, self.my_intersection.corner_bottom_right[1] + 6)
        
        #left of path
        p3 = (self.my_intersection.corner_bottom_right[0] - 3.5/2 -2, self.my_intersection.corner_bottom_right[1] + 5)
        #right of path
        p4 = (self.my_intersection.corner_bottom_right[0] - 3.5/2 +1, self.my_intersection.corner_bottom_right[1] + 2)
        p1time = 4/speed * -1
        p2time = 6/speed * -1
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"south"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"south"),p2time)
        p3time = utils.pyth_distance(p3,(self.my_intersection.corner_bottom_right[0]-3.5/2,self.my_intersection.corner_bottom_right[1]))/speed
        p3time = -1 * p3time
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"south"),p3time)
        p4time = utils.pyth_distance(p4,(self.my_intersection.corner_bottom_right[0]-3.5/2,self.my_intersection.corner_bottom_right[1]))/speed
        p4time = -1 * p4time
        #print(self.my_intersection.vehicle_lies_outside_intersection_square(p4))
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"south"),p4time)

        #case when vehicle is coming from east and doing straight ahead
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"east"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"east"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"east"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"east"),p4time)

        #case when vehicle is coming from north and doing straight ahead
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"north"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"north"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"north"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"north"),p4time)

        #case when vehicle is coming from west and doing straight ahead
        #make vehicles:
        #directly on path
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of path
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of path
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"west"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"west"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"west"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.STRAIGHT_AHEAD,"west"),p4time)

    def left_turn_test(self):
        #case when vehicle is coming from south and doing a left turn
        #courses = [IntersectionCourses.LEFT_TURN] * 4 #all vehicles are trying a left turn
        #ved = ["south"] * 4
        speed = 3.2
        interval = 3

        #make vehicles:
        #directly on arc:
        p1 = (4.933386259-3.5, 1.795605752-3.5,0)
        p2 = (1.795605752-3.5, 4.933386259-3.5,0)
        #left of arc
        p3 = (1.931851653-3.5, 0.5176380902 -3.5, 0) #r = 2, theta = 15deg
        #right of arc
        p4 = (1.330570426 - 3.5, 5.336626495 - 3.5) #r = 5.5, theta = 76

        p1time = -(1.832595715/speed)
        p2time = -(6.414085001079161/speed)
        p3time = -(0.5235987755982988/speed)
        p4time = -(7.295476273336298/speed)

        
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"),p4time)
        
        #case when vehicle is coming from east and doing a left turn
        #make vehicles:
        #directly on arc:
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of arc
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of arc
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"east"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"east"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"east"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"east"),p4time)
        
        #case when vehicle is coming from north and doing a left turn
        #make vehicles:
        #directly on arc:
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of arc
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of arc
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"north"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"north"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"north"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"north"),p4time)

        #case when vehicle is coming from west and doing a left turn
        #make vehicles:
        #directly on arc:
        p1 = utils.rotate_point(self.my_intersection.center,p1,math.pi/2)
        p2 = utils.rotate_point(self.my_intersection.center,p2,math.pi/2)
        #left of arc
        p3 = utils.rotate_point(self.my_intersection.center,p3,math.pi/2)
        #right of arc
        p4 = utils.rotate_point(self.my_intersection.center,p4,math.pi/2)
        #print(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"south"))
        self.assertAlmostEqual(E_estimate.project_vehicle(p1,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"west"),p1time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p2,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"west"),p2time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p3,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"west"),p3time)
        self.assertAlmostEqual(E_estimate.project_vehicle(p4,speed,self.my_intersection,interval,IntersectionCourses.LEFT_TURN,"west"),p4time)

#these functions are not unit tested because they are so primitive :x
#line_intersection(line1, line2):
#    def det(a, b):



if __name__     == 'main':
    unittest.main()