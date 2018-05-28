from collections import OrderedDict
from particle_filter import ParticleFilter
import plotter
import time
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
plot_folder = os.path.join(dir_path, "plotfolder")
import Intersection
from threading import Lock

class RiskEstimator:

    def __init__(self, n_particles, intersection, initial_measurements, pose_covariance, speed_deviation, init_time,mutex,plot=False):

        self.intersection = intersection
        self.plot = plot #boolean
        self.last_t = init_time
        self.mutex = mutex

        #get initial directions that the vehicle are travelling towards
        travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in initial_measurements]

        #if initial measurements are not provided. particle filter is not initialized.
        #this is useful when cloning as particle filter instance variable is later changed
        #by copying over from the ones to be copied
        if initial_measurements == []:
            return

        #initialize the particle filters
        self.particle_filters = \
            [ParticleFilter(travelling_directions, self.intersection, n_particles, m, pose_covariance, speed_deviation) \
                for m in initial_measurements]

    def clone(self):
        self.mutex.acquire()
        cloned_object = RiskEstimator( \
            400,Intersection.Intersection(),\
            [],\
            None,\
            None,\
            self.last_t,\
            Lock(),\
            False)
        cloned_object.particle_filters = [x.clone() for x in self.particle_filters]
        self.mutex.release()
        return cloned_object

    #todo this is quite ugly, fix this
    #todo save ego-id in RiskEstimator class
    # sets override intention
    def setKnownIc(self, id, Ic):
        self.mutex.acquire()
        self.particle_filters[id].setKnownIc(Ic)
        self.mutex.release()
    def removeKnownIc(self, id):
        self.mutex.acquire()
        self.particle_filters[id].removeKnownIc()
        self.mutex.release()

    def setKnownIs(self, id, Is):
        self.mutex.acquire()
        self.particle_filters[id].setKnownIs(Is)
        self.mutex.release()
    def removeKnownIs(self, id):
        self.mutex.aquire()
        self.particle_filters[id].removeKnownIs()
        self.mutex.release()


    def update_state(self, t, measurements):
        self.mutex.acquire()
        
        most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]

        interval = t - self.last_t

        for i, pfilter in enumerate(self.particle_filters):
            pfilter.step_time(i, measurements[i], most_likely_states, interval)
        
        self.last_t = t

        if self.plot:
            plotter.plot_particles(self.particle_filters, measurements, t, plot_folder)
        
        self.mutex.release()


    # probability(Es = "go")
    def getExpectation(self, id):
        self.mutex.acquire()
        density =  self.particle_filters[id].Es_density["go"]
        self.mutex.release()
        return density

    def get_risk(self):
        risks = []
        self.mutex.acquire()
        for pfilter in self.particle_filters:
            risk = 0
            for i,s in enumerate(pfilter.particles):
                weight = pfilter.weights[i]
                
                if ((s.Es == "stop" and s.Is == "go") or (s.Es == "go" and s.Is == "stop")):
                    risk += weight
            
            risks.append(risk)
        
        self.mutex.release()
        return risks

    def get_expectation_Es(self,vehicle_id,intended_course=None):
        #compute expectation of Es:
        # if intended_course given, compute p(ic != es | ic = x)
        # useful to measure expectation on a certain maneuver

        expectation = 0.
        for pfilter in self.particle_filters:
            for i,s in enumerate(pfilter.particles):
                weight = pfilter.weights[i]
                if (intended_course is not None):
                    if (intended_course == s.Ic):
                        expectation = expectation + (s.Es*weight)
                else:
                    expectation = expectation + (s.Es*weight)

        return expectation

    def get_future_expectation_Es(self,vehicle_id,intended_course=None,interval=None):
        """
        Compute mathematical expecation of Es variable for a future time.
        This is useful because in the maneuver negotiator,
        when a request is received by a car, it computes the no-conflict procedure
        that checks if there is no conflict with the sender in future. 

        This calculation depends no  what the sender is supposed to do at the
        intersection _when_ the answer for his request is received. as receiving 
        takes an upper bound time on it, we have to compute what the car is supposed
        to do in a future time when he receives this grant/deny request.

        for that to happen, we have to compute expectation of Es for the future.

        Our current assumptions are that the intention and courses of all the other cars continue
        to progress as following the intention of the car in this future.
        so we have to progress the markov chain by projecting the speeds and velocities
        as per the current intetions and then computing the expectation in this future point.

        This future point is the upper bound on the time for the packet to be received
        by the sender + a constant number that will be the time to compute this whole function.
        and also time for the sender to receive all messages from that it require.

        """

        if (interval==None):
            interval = 0.4 # todo: make interval depend on actual values on the upperbound of delay
        
        """
        algorithm:
        - make a copy of the whole markov model of the driver.py
        - get speed and location of where the vehicles will be after the interval.
        - use that to do an update step with measurement as speed an location as above.
        - call get_expecation_es on this markov model with sender


        """
        current_hmm_model = self.clone()

        measurement_vector = []
        for vehicle in current_hmm_model.particle_filters:
            #calculate where vehicles will be after one interval:

            #build measurement vector
            pass
        current_hmm_model.update_state(interval+current_hmm_model.last_t,measurement_vector)
        expectation = current_hmm_model.get_expectation_Es(vehicle_id,intended_course)
        return expectation

    
    def get_copy(self,vehicle_id):
        self.mutex.acquire()

        particles = self.particle_filters[vehicle_id].deep_copy_particles()
        weights = self.particle_filters[vehicle_id].weights

        self.mutex.release()
        return [particles,weights]
