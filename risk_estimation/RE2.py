import sys
sys.path.append("..")
from utils.Intersection import *
from math import *
import scipy.stats as sp

def normal_cdf(x, mu, sigma):
    q = math.erf((x-mu) / (sigma*math.sqrt(2.0)))
    return (1.0 + q) / 2.0

class RE2:
    def __init__(self, td):
        self.travelling_directions = td
        self.nr_cars = len(td)
        self.intersection = Intersection()

        self.intentionDensities = {}  
        self.expectationDensities = {}
        self.turns = self.intersection.turns

        self.Ldict = {(car,Ic,Is):[] for car in range(self.nr_cars) for Ic in self.turns for Is in ["go", "stop"]}

        default_I_dens = 1.0/(len(self.turns)*2)
        default_E_dens = 0.5


        for id in range(self.nr_cars):
            II = {}
            for Ic in self.turns:
                for Is in ["go", "stop"]:
                    II[Ic,Is] = default_I_dens
                self.expectationDensities[id, Ic] = default_E_dens
                
            self.intentionDensities[id] = II
    
    def update_state(self, t, ms, deviations):
        #intention
        for car in range(self.nr_cars):
            D = {}
            for turn in self.turns:
                for i in ["go", "stop"]:
                    thisL = 1.0/self.I_error(ms[car], deviations[car], self.travelling_directions[car], turn, i)
                    
                    list = self.Ldict[car,turn,i]
                    list.append(thisL)
                    if len(list) > 2:
                        del list[0]
                    
                    L = sum([w*l for w,l in zip(list, range(1,len(list)+1))])

                    e = self.expectationDensities[car,turn]
                    ee = e if i == "go" else 1-e
                    L *= 0.4 + 1.2 * ee
                    D[turn,i] = L
            
            s = sum(D.values())
            for key,value in D.iteritems():
                D[key] = value/s
        
            self.intentionDensities[car] = D
        
        T = {}

        for car in range(self.nr_cars):
            trav_dir = self.travelling_directions[car]
            x,y,theta,speed = ms[car]
            xd, yd, td, sd = deviations[car]
            for turn in self.turns:
                c = self.intersection.courses[trav_dir,turn]
                
                tt = c.rotate(*c.getPose(c.getDistance(*c.rotate(x,y,theta))), dir=-1)[2]
                px, py = x+xd*0.8*cos(tt),y+yd*0.8*sin(tt)

                if c.getDistance(*c.rotate(px,py,0)) > c.distance_to_crossing:
                    ps = speed - sd*0.8-0.5
                else:
                    ps = speed + sd*0.8+0.5

                

                mx, my = x-xd*0.8*cos(tt),y-yd*0.8*sin(tt)

                if c.getDistance(*c.rotate(mx,my,0)) > c.distance_to_crossing:
                    m_s = speed + sd*0.8+0.5
                else:
                    m_s = speed - sd*0.8-0.5

                a = c.getTimeToCrossing2(px, py,theta,ps, Is="go")
                b = c.getTimeToCrossing2(mx, my,theta,m_s, Is="go")

                m = (a+b)/2

                s = abs(m-a)


                #print car, turn, m, s
                T[car, turn] = m, s
        

        E = {}

        for egocar in range(self.nr_cars):
            td_ego = self.travelling_directions[egocar]
            for turn_ego in self.turns:
                min_es = 1.0
                for othercar in range(self.nr_cars):
                    if egocar == othercar:
                        continue
                    td_other = self.travelling_directions[othercar]

                    if self.intersection.hasRightOfWay(td_ego, turn_ego, td_other):
                        continue
                    
                    e_sum = 0
                    mean_ego, std_ego = T[egocar, turn_ego]
                    for turn_other in self.turns:
                        mean_other, std_other = T[othercar, turn_other]
                        gap_mean, gap_std = mean_other - mean_ego, sqrt(std_ego**2 + std_other**2)
                        
                        p_gap_enough = 1 - (normal_cdf(5, gap_mean, gap_std) - \
                                       normal_cdf(-2, gap_mean, gap_std))
                        
                        e_sum += p_gap_enough * (self.intentionDensities[othercar][turn_other, "go"] + \
                                                    self.intentionDensities[othercar][turn_other, "stop"])
                    
                    if e_sum < min_es:
                        min_es = e_sum
                
                E[egocar,turn_ego] = min_es
                
        self.expectationDensities = E


    def I_error(self, m, dev, td, turn, i):
        c = self.intersection.courses[td, turn]
        opt = getOpt(c, m, i)
        return np.sum(er2(np.array(m), np.array(dev), np.array(opt),np.array([125,125,125,1])))

    
    def getRisk(self, car):

        sum = 0
        for turn in self.turns:
            #es=go and is = stop
            e = self.expectationDensities[car, turn]
            sum += e * self.intentionDensities[car][turn, "stop"]

            sum += (1-e) * self.intentionDensities[car][turn, "go"]

        return sum
        
        


def getOpt(c, (x,y,theta,speed), Is):
    x,y,theta = c.rotate(x,y,theta)
    d = c.getDistance(x,y,theta)
    if d > c.distance_to_crossing and Is=="stop":
        px,py,ptheta = (3.25, -7.5, pi/2)
    else:
        px, py, ptheta = c.getPose(d)
    ps = c.sp_go.getSpeed(d) if Is=="go" else c.sp_stop.getSpeed(d)
    px,py,ptheta = c.rotate(px, py, ptheta, dir = -1)
    return (px, py, ptheta, ps)


def er2(m,dev,d,w):
    return w * ((d-m)**2+dev**2)