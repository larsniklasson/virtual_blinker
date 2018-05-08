#!/usr/bin/env python



from math import *
import spline as s
#from auto_master import *
from geometry import *

class ErrorCalc:
    def __init__(self, path):

        self.path = map(lambda p: Point(*p), path)
        self.line = (self.path.pop(0), self.path.pop(0))
        

    def calculateError(self, (x, y)):
        p = Point(x,y)
        
        l1, l2 = self.line
        
        if l2 == None:
            return (0,0)
            
            
        while hasPassedLine(p, (l1, l2)):
            if len(self.path) == 0:
                self.line = (l2, None)
                return (0,0)
            
            tl1 = l2    
            tl2 = self.path.pop(0)
            self.line= (tl1, tl2)
            l1, l2 = self.line

        
        value = getDistanceFromLine(p, self.line)
        
        
        if isLeftOfLine(p, self.line):
            
            #+1 to account for points in self.line
            return (value, len(self.path) + 1)
            
        else:
            return (-value, len(self.path) + 1)

