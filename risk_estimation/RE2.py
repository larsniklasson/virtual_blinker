import sys
sys.path.append("..")
from utils.Intersection import *
from math import *
import scipy.stats as sp
from threading import Lock
from config import *

ww = [None, 3.25, 11.75, pi*11.75/4.0, pi*11.75/2.0 - 3.25]
turnproj = {"left": [1, 3, 4], "right": [1], "straight": [1, 2]}

def normal_cdf(x, mu, sigma):
    q = math.erf((x-mu) / (sigma*math.sqrt(2.0)))
    return (1.0 + q) / 2.0

class RE2:
    def __init__(self, td):
        self.lock = Lock()
        self.travelling_directions = td
        self.nr_cars = len(td)
        self.intersection = Intersection()

        self.eb = [False]*self.nr_cars
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
    
    def update_state(self, t, ms, deviations, blinker, eb):
        with self.lock:
            self.eb = eb
            self.latest_ms = ms
            #intention
            for car in range(self.nr_cars):
                D = {}
                for turn in self.turns:
                    for i in ["go", "stop"]:
                        thisL = 1.0/self.I_error(ms[car], deviations[car], self.travelling_directions[car], turn, i)
                        
                        list = self.Ldict[car,turn,i]
                        list.append(thisL)
                        if len(list) > 3:
                            del list[0]
                        
                        L = sum([w*l for w,l in zip(list, range(1,len(list)+1))])

                        e = self.expectationDensities[car,turn]
                        ee = e if i == "go" else 1-e
                        L *= 0.4 + 1.2 * ee

                        if blinker[car] == turn:
                            L *= 5

                        D[turn,i] = L
                
                s = sum(D.values())
                for key,value in D.iteritems():
                    D[key] = value/s
            
                self.intentionDensities[car] = D
            
            self.T = {}

            for car in range(self.nr_cars):
                trav_dir = self.travelling_directions[car]
                x,y,theta,speed = ms[car]
                xd, yd, td, sd = deviations[car]
                for turn in self.turns:
                    c = self.intersection.courses[trav_dir,turn]
                    
                    tt = c.rotate(*c.getPose(c.getDistance(*c.rotate(x,y,theta))), dir=-1)[2]
                    px, py = x+xd*0.8*cos(tt),y+yd*0.8*sin(tt)

                    
                    mx, my = x-xd*0.8*cos(tt),y-yd*0.8*sin(tt)

                    for i in turnproj[turn]:
                                            

                        if c.getDistance(*c.rotate(px,py,0)) > c.distance_to_crossing:
                            ps = speed - sd*0.8-0.5
                        else:
                            ps = speed + sd*0.8+0.5

                        if c.getDistance(*c.rotate(mx,my,0)) > c.distance_to_crossing:
                            m_s = speed + sd*0.8+0.5
                        else:
                            m_s = speed - sd*0.8-0.5

                        a = c.getTimeToCrossing2(px, py,theta,ps, Is="go", extra=ww[i])
                        b = c.getTimeToCrossing2(mx, my,theta,m_s, Is="go", extra=ww[i])

                        m = (a+b)/2

                        s = abs(m-a)

                        self.T[car, turn, i] = m, s
            

            E = {}

            self.G = {}

            for egocar in range(self.nr_cars):
                td_ego = self.travelling_directions[egocar]
                for turn_ego in self.turns:
                    x,y,theta,speed = ms[egocar]
                    c = self.intersection.courses[td_ego, turn_ego]
                    if c.hasLeftIntersection(x,y,theta):
                        E[egocar, turn_ego] = 1.0
                        continue
                    
                    min_es = 1.0
                    for othercar in range(self.nr_cars):
                        
                        if egocar == othercar:
                            continue

                        td_other = self.travelling_directions[othercar]

                        if self.intersection.hasRightOfWay(td_ego, turn_ego, td_other):
                            continue
                        
                        e_sum = 0
                        for turn_other in self.turns:
                            
                            if not self.intersection.doesCoursesIntersect(td_ego, turn_ego, td_other, turn_other):
                                p_gap_enough = 1
                            else:
                                ii,jj = self.intersection.getIJ(td_ego, turn_ego, td_other, turn_other)

                                mean_ego, std_ego = self.T[egocar, turn_ego, ii]

                                mean_other, std_other = self.T[othercar, turn_other, jj]
                                gap_mean, gap_std = mean_other - mean_ego, sqrt(std_ego**2 + std_other**2)
                                
                                self.G[egocar, turn_ego, othercar, turn_other] = gap_mean, gap_std
                                p_gap_enough = 1 - (normal_cdf(3, gap_mean, gap_std) - \
                                            normal_cdf(-1, gap_mean, gap_std))
                                
                            e_sum += p_gap_enough * self.intentionCarTurn(othercar, turn_other)
                            
                        if e_sum < min_es:
                            min_es = e_sum
                    
                    E[egocar,turn_ego] = min_es
                    
            self.expectationDensities = E


    def I_error(self, m, dev, td, turn, i):
        c = self.intersection.courses[td, turn]
        opt = getOpt(c, m, i)
        return np.sum(er2(np.array(m), np.array(dev), np.array(opt),np.array([125,125,125,1])))


    def getRisk2(self, ego_car, risk_car):
        with self.lock:
            td_ego = self.travelling_directions[ego_car]
            td_risk = self.travelling_directions[risk_car]
            sum = 0
            for ego_turn in self.turns:
                for risk_turn in self.turns:
                    if self.intersection.doesCoursesIntersect(td_ego, ego_turn, td_risk, risk_turn):
                        eb = self.eb[risk_car]
                        if eb:
                            ii,jj = self.intersection.getIJ(td_risk, risk_turn, td_ego, ego_turn)

                            tti_risk,_ = self.T[risk_car, risk_turn, ii]
                            tti_ego,_ = self.T[ego_car, ego_turn, jj]
                            
                            if tti_risk > -1 and tti_risk < 1 and tti_ego < 2 and tti_ego > -1:
                                sum += self.intentionCarTurn(risk_car, risk_turn) * self.intentionCarTurn(ego_car, ego_turn)


                        e = self.expectationDensities[risk_car, risk_turn]
                        if not self.intersection.hasRightOfWay(td_risk, risk_turn, td_ego):
                            try:
                                gap_mean, gap_std = self.G[risk_car, risk_turn, ego_car, ego_turn]
                                p_gap_not_enough = (normal_cdf(2, gap_mean, gap_std) - \
                                        normal_cdf(-0.5, gap_mean, gap_std))
                                
                                sum += (1-e) * self.intentionDensities[risk_car][risk_turn, "go"] * \
                                    self.intentionCarTurn(ego_car, ego_turn)\
                                    * p_gap_not_enough
                            except:
                                pass

            return sum

    def checkVehicleInFront(self, ego_car):
        with self.lock:
            d_min = 999999999
            ego_ms = self.latest_ms[ego_car]
            ego_td = self.travelling_directions[ego_car]
            for ego_turn in self.turns:
                ego_course = self.intersection.courses[ego_td, ego_turn]
                ego_P = self.intentionCarTurn(ego_car, ego_turn)
                if ego_P < 0.15:
                    continue

                for other_car in range(self.nr_cars):
                    if other_car == ego_car:
                        continue
                    other_ms = self.latest_ms[other_car]
                    other_td = self.travelling_directions[other_car]
                    for other_turn in self.turns:
                        other_P = self.intentionCarTurn(other_car, other_turn)
                        if other_P < 0.15:
                            continue
                        ego_hasLeft = ego_course.hasLeftIntersection(*ego_ms[:3])
                        if ego_td == other_td and (not ego_hasLeft) or ego_hasLeft and self.intersection.merge(ego_td, ego_turn, other_td, other_turn):
                            
                            d_ego = ego_course.getDistance(*ego_ms[:3], rotate=True)
                            d_other = ego_course.getDistance(*other_ms[:3], rotate=True)
                            d = d_other - d_ego
                            if d < d_min and d > 0:
                                d_min = d
                                speed = other_ms[-1]
        
            if d_min < 5 + 2 * ego_ms[-1]:
                return speed #TODO return lower speed if closer
            else:
                return -1


    """
    def getRisk(self, car):
        with self.lock:
            sum = 0
            for turn in self.turns:
                e = self.expectationDensities[car, turn]
                #sum += e * self.intentionDensities[car][turn, "stop"]
                sum += (1-e) * self.intentionDensities[car][turn, "go"]

            return sum
    """

    def noConflict(self, car, turn):
        with self.lock:
            return self.expectationDensities[car, turn] > RISK_CONFIG["grant_threshold"]
        
        
    def intentionCarTurn(self, car, turn):
        a = self.intentionDensities[car]
        return a[turn, "go"] + a[turn, "stop"]

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