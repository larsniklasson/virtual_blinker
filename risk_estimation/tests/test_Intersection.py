import unittest
import sys
sys.path.append("..")
from Intersection import Intersection
from Intersection import IntersectionType

class IntersectionTest(unittest.TestCase):
    def setUp(self):
        self.center_point = (0,0) # change as needed
        self.lane_width = 3.5
        self.my_intersection = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'north-south')
    def test_Orthogonality(self):
        #for now, the points corner points in intersection are orthogonal
        print(self.my_intersection)

    def test_Vehicle_lies_outside_intersection_square(self):
        #test if given vehicle lies outside intersection square

        #test vehicles by putting outside square
        v1 = (6,0,0)
        v2 = (0,6,0)
        v3 = (-6,0,0)
        v4 = (0,-6,0)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v1), True)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v2), True)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v3), True)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v4), True)

        #test vehicles by putting inside square
        v1 = (0,0,0)
        v2 = (1,2,0)
        v3 = (-3,-3,0)
        v4 = (3.5,-3.5,0)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v1), False)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v2), False)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v3), False)
        self.assertEqual(self.my_intersection.vehicle_lies_outside_intersection_square(v4), False)


    def test_vehicle_lies_eastwest(self): #checks if the vehicle given lies in the horizontal road
    #  also tests vehicle_lies_northsouth(self,vehicle):
        #eastwest
        v1 = (6,0,0)
        v2 = (3.6,0,0)
        v3 = (-3.6,0,0)
        v4 = (-10,2.5,0)
        v5 = (10,-3,0)
        v6 = (6,0,0)
        v7 = (0,0,0)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v1),True)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v2),True)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v3),True)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v4),True)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v5),True)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v6),True)

        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v1),False)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v2),False)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v3),False)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v4),False)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v5),False)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v6),False)

        #northsouth:
        v1 = (0,6,0)
        v2 = (0,3.6,0)
        v3 = (0,-3.6,0)
        v4 = (0,-10,0)
        v5 = (0,10,0)
        v6 = (0,6,0)
        v7 = (0,0,0)
        
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v1),False)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v2),False)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v3),False)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v4),False)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v5),False)
        self.assertEqual(self.my_intersection.vehicle_lies_eastwest(v6),False)
        
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v1),True)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v2),True)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v3),True)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v4),True)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v5),True)
        self.assertEqual(self.my_intersection.vehicle_lies_northsouth(v6),True)
    def test_right_of_way(self):
        #northsouth
        ns = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'north-south')
        ew = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'east-west')

        v1 = (-1.75,4,0)
        v2 = (-1.75,5,0)
        v3 = (-1.75,9,0)
        
        v4 = (4,1.75,0)
        v5 = (5,1.75,0)
        v6 = (9,1.75,0)

        v7 = (1.75,-4,0)
        v8 = (1.75,-5,0)
        v9 = (1.75,-9,0)

        v10 = (-4,-1.75,0)
        v11 = (-5,-1.75,0)
        v12 = (-9,-1.75,0)
        ved = ['north','north','north','east','east','east','south','south','south','west','west','west']

        vehicle_set = [v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12]

        self.assertEqual(ns.right_of_way(vehicle_set,0,ved),[3,4,5,9,10,11])
        self.assertEqual(ns.right_of_way(vehicle_set,1,ved),[3,4,5,9,10,11])
        self.assertEqual(ns.right_of_way(vehicle_set,2,ved),[3,4,5,9,10,11])

        self.assertEqual(ns.right_of_way(vehicle_set,3,ved),[])
        self.assertEqual(ns.right_of_way(vehicle_set,4,ved),[])
        self.assertEqual(ns.right_of_way(vehicle_set,5,ved),[])

        self.assertEqual(ns.right_of_way(vehicle_set,6,ved),[3,4,5,9,10,11])
        self.assertEqual(ns.right_of_way(vehicle_set,7,ved),[3,4,5,9,10,11])
        self.assertEqual(ns.right_of_way(vehicle_set,8,ved),[3,4,5,9,10,11])

        self.assertEqual(ns.right_of_way(vehicle_set,9 ,ved),[])
        self.assertEqual(ns.right_of_way(vehicle_set,10,ved),[])
        self.assertEqual(ns.right_of_way(vehicle_set,11,ved),[])

        #east-west:
        self.assertEqual(ew.right_of_way(vehicle_set,0,ved),[])
        self.assertEqual(ew.right_of_way(vehicle_set,1,ved),[])
        self.assertEqual(ew.right_of_way(vehicle_set,2,ved),[])

        self.assertEqual(ew.right_of_way(vehicle_set,3,ved),[0,1,2,6,7,8])
        self.assertEqual(ew.right_of_way(vehicle_set,4,ved),[0,1,2,6,7,8])
        self.assertEqual(ew.right_of_way(vehicle_set,5,ved),[0,1,2,6,7,8])

        self.assertEqual(ew.right_of_way(vehicle_set,6,ved),[])
        self.assertEqual(ew.right_of_way(vehicle_set,7,ved),[])
        self.assertEqual(ew.right_of_way(vehicle_set,8,ved),[])

        self.assertEqual(ew.right_of_way(vehicle_set,9 ,ved), [0,1,2,6,7,8])
        self.assertEqual(ew.right_of_way(vehicle_set,10,ved),[0,1,2,6,7,8])
        self.assertEqual(ew.right_of_way(vehicle_set,11,ved),[0,1,2,6,7,8])

        """
        print("right of way = " + str(ns.right_of_way(vehicle_set,2)))
        print("vehicle lies east west = " + str(ns.vehicle_lies_eastwest(vehicle_set[2])))
        a  = []
        for i,v in enumerate(vehicle_set):
            print(ns.vehicle_lies_eastwest(v))
            if (ns.vehicle_lies_eastwest(v)):
                a.append(i)

        print(a)
        """
        #eastwest
        
        #ew = Intersection(self.center_point,3.5,IntersectionType.GIVE_WAY_4,'nort-south')

    def test_find_vehicle_pos(self):
        v1 = (-1.75,4,0)
        v2 = (4,1.75,0)
        v3 = (1.75,-4,0)
        v4 = (-4,-1.75,0)

        self.assertEqual(self.my_intersection.find_vehicle_pos(v1),('north',(- (self.lane_width/2), self.lane_width)))
        self.assertEqual(self.my_intersection.find_vehicle_pos(v2),('east',( self.lane_width, (self.lane_width/2))))
        self.assertEqual(self.my_intersection.find_vehicle_pos(v3),('south',(self.lane_width/2,-self.lane_width)))
        self.assertEqual(self.my_intersection.find_vehicle_pos(v4),('west',(-self.lane_width, (-(self.lane_width/2)))))

        pass


if __name__     == 'main':
    unittest.main()