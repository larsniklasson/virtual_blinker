from collections import OrderedDict
from particle_filter import ParticleFilter
import plotter
import time
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
plot_folder = os.path.join(dir_path, "plotfolder")

class RiskEstimator:

    def __init__(self, n_particles, intersection, initial_measurements, pose_covariance, speed_deviation, init_time, plot=False):

        self.intersection = intersection
        self.plot = plot #boolean
        self.last_t = init_time

        #get initial directions that the vehicle are travelling towards
        travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in initial_measurements]

        #initialize the particle filters
        self.particle_filters = \
            [ParticleFilter(travelling_directions, self.intersection, n_particles, m, pose_covariance, speed_deviation) \
                for m in initial_measurements]


    #todo this is quite ugly, fix this
    #todo save ego-id in RiskEstimator class
    # sets override intention
    def setKnownIc(self, id, Ic):
        self.particle_filters[id].setKnownIc(Ic)
    def removeKnownIc(self, id):
        self.particle_filters[id].removeKnownIc()

    def setKnownIs(self, id, Is):
        self.particle_filters[id].setKnownIs(Is)
    def removeKnownIs(self, id):
        self.particle_filters[id].removeKnownIs()


    def update_state(self, t, measurements):
        
        most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]

        interval = t - self.last_t

        for i, pfilter in enumerate(self.particle_filters):
            pfilter.step_time(i, measurements[i], most_likely_states, interval)
        
        self.last_t = t

        if self.plot:
            plotter.plot_particles(self.particle_filters, measurements, t, plot_folder)


    # probability(Es = "go")
    def getExpectation(self, id):
        return self.particle_filters[id].Es_density["go"]

    def get_risk(self):
        risks = []
        for pfilter in self.particle_filters:
            risk = 0
            for i,s in enumerate(pfilter.particles):
                weight = pfilter.weights[i]
                
                if ((s.Es == "stop" and s.Is == "go") or (s.Es == "go" and s.Is == "stop")):
                    risk += weight
            
            risks.append(risk)
        
        return risks