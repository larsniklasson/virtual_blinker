from collections import OrderedDict
from particle_filter import ParticleFilter
import plotter
import csv
import time

class RiskEstimator:

    def __init__(self, n_particles, intersection, initial_measurements, pose_covariance, speed_deviation, init_time, plot=False, plot_folder=None):

        self.intersection = intersection
        self.plot_folder = plot_folder
        self.plot = plot
        self.last_t = init_time
        travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in initial_measurements]
        
        self.particle_filters = \
            [ParticleFilter(travelling_directions, self.intersection, n_particles, m, pose_covariance, speed_deviation) \
                for m in initial_measurements]

        nr_cars = len(initial_measurements)
        self.known_Is = {i: None for i in range(nr_cars)}
        self.known_Ic = {i: None for i in range(nr_cars)}


    #Todo maybe remove this, perhaps unneccassary
    def setKnownIc(self, id, Ic):
        self.known_Ic[id] = Ic

    def removeKnownIc(self, id):
        self.known_Ic[id] = None

    def setKnownIs(self, id, Is):
        self.known_Is[id] = Is

    def removeKnownIs(self, id):
        self.known_Is[id] = None


    def update_state(self, t, measurements):
        
        most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]

        interval = t - self.last_t

        for i, pfilter in enumerate(self.particle_filters):
            pfilter.step_time(i, measurements[i], most_likely_states, interval, self.known_Ic[i], self.known_Is[i])
        
        self.last_t = t

        risks, expectations = self.get_risk_and_expectations()

        if (self.plot):
            plotter.plot_particles(self.particle_filters, measurements, t, risks, self.plot_folder)


    def getExpectation(self, id):
        return self.particle_filters[id].Es_density["go"]

    def get_risk_and_expectations(self):
        risks = []
        expectations = []
        for pfilter in self.particle_filters:
            risk = 0
            expectation = 0
            for i,s in enumerate(pfilter.particles):
                weight = pfilter.weights[i]
                
                if s.Es == "go": 
                    expectation += weight

                if ((s.Es == "stop" and s.Is == "go") or (s.Es == "go" and s.Is == "stop")):
                    risk += weight
            
            risks.append(risk)
            expectations.append(expectation)
        
        return risks, expectations