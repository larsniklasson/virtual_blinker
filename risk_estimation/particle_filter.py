from copy import deepcopy
import numpy as np
import utils_re
from E_estimate import Es_estimate
from I_estimate import Is_estimate, Ic_estimate
from PS_estimate import *
import math

from scipy.stats import multivariate_normal
from scipy.stats import norm

class ParticleFilter:
    def __init__(self,travelling_directions, intersection, n_particles, initial_measurement, pose_covariance, speed_deviation):
        
        self.weights = []
        
        self.n_particles = n_particles
        self.travelling_directions  = travelling_directions
        self.intersection = intersection

        self.position_covariance =  pose_covariance[np.ix_([0,2],[0,2])]

        self.theta_deviation = pose_covariance[2][2]
        self.pose_covariance  = pose_covariance
        self.speed_deviation = speed_deviation


        self.posterior_particles = [] 
        self.posterior_weights = []

        self.neff_threshold = self.n_particles / 2

        P = initial_measurement[:3]
        S = initial_measurement[-1]

        self.particles = utils_re.generate_inital_particles(intersection, P, S, n_particles, self.pose_covariance, self.speed_deviation)
        self.weights = [1.0 / self.n_particles] * self.n_particles


    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    
    def get_most_likely_state(self):
        
        blob = zip(self.particles, self.weights)

        Es_stop_weights_sum = sum([w for (p, w) in blob if p.Es == "stop"])
        new_Es = "stop" if Es_stop_weights_sum > 0.5 else "go"

        Is_stop_weights_sum = sum([w for (p, w) in blob if p.Is == "stop"])
        new_Is = "stop" if Is_stop_weights_sum > 0.5 else "go"


        turn_weights = {t : 0 for t in self.intersection.turns}
        for p,w in blob:
            turn_weights[p.Ic] += w
        new_Ic = max(turn_weights, key=turn_weights.get)


        i = np.argmax(self.weights)
        new_P = self.particles[i].P
        new_S = self.particles[i].S

        return StateVector(new_Es,new_Is,new_Ic,new_P,new_S)

    def step_time(self, id, measurement_vector, most_likely_states, interval, Is_override, Ic_override):
        new_particles = []
        for p in self.particles:

            new_Es = Es_estimate(id, p, self.travelling_directions, self.intersection, most_likely_states)

            if Is_override:
                new_Is = Is_override
            else:
                new_Is = Is_estimate(p.Is, new_Es)

            if Ic_override:
                new_Ic = Ic_override
            else:
                new_Ic = Ic_estimate(p.Ic, self.intersection.turns)

            new_P, new_S = PS_estimate(p, self.travelling_directions[id], self.intersection, interval, self.pose_covariance, self.speed_deviation)
            new_particles.append(StateVector(new_Es ,new_Is, new_Ic, new_P, new_S))
            
        
        new_weights = [self.likelihood(p, measurement_vector) for p in self.particles]


        
        #normalize
        w_sum = float(sum(new_weights))
        new_weights = [w/w_sum for w in new_weights]

        #resample particles by their weight:W
        #sources obtained: 
        # for neff comparision: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
        # This is a sampling importance resampling. 
        # http://people.math.aau.dk/~kkb/Undervisning/Bayes14/sorenh/docs/sampling-notes.pdf
        if self.neff(new_weights) < self.neff_threshold:
            indices_resample = np.random.choice(range(self.n_particles), self.n_particles, p = new_weights)
            new_particles = [new_particles[i] for i in indices_resample]
            new_weights = [new_weights[i] for i in indices_resample]

        self.particles = new_particles
        self.weights = new_weights
            

        
        

    
    def likelihood(self,p, measurement_vector):
        #used this: https://blogs.sas.com/content/iml/2010/12/10/converting-between-correlation-and-covariance-matrices.html
        
        #in the likelihood of pose, the x and y are linear, and pdf calculation for a normal distribution is okay
        # but theta is periodic. angles repeat after 2pi. 
        # if there is a distribution with mean , say 350, under a non periodic normal distribution,
        # obtaining 0 is quite unlikely. but as theta is periodic and 0 angle is just 5 degrees away from 350
        # we would expect a higher likelihood for that.
        # solution would be to use a wrapped normal distribution as per : https://en.wikipedia.org/wiki/Wrapped_normal_distribution
        # this maybe computationaly expensive and right now i am using a simplified version of it.

        #https://math.stackexchange.com/questions/892832/why-we-consider-log-likelihood-instead-of-likelihood-in-gaussian-distribution
        #calculating this likelihood takes a lot of time as per benchmark profile output
        #https://stats.stackexchange.com/questions/201545/likelihood-calculation-in-particle-filtering
        mean_xy = p.P[:2]
        mean_theta = p.P[-1]
        mean_speed = p.S

        xy = measurement_vector[:2]
        theta = measurement_vector[2]
        speed = measurement_vector[3]

        position_likelihood = multivariate_normal.pdf(xy,mean = mean_xy, cov=self.position_covariance)
        
        upper_bound = mean_theta + math.pi
        lower_bound = mean_theta - math.pi

        while (theta < lower_bound or theta > upper_bound):
            if theta < lower_bound:
                theta += math.pi*2
            if theta > upper_bound:
                theta -= math.pi*2


        theta_likelihood = norm.pdf(theta, loc=mean_theta, scale=self.theta_deviation)

        speed_likelihood = norm.pdf(speed, loc=mean_speed, scale=self.speed_deviation)

        pose_likelihood = position_likelihood * theta_likelihood # position and angle are independent so this multiplication is okay

        return pose_likelihood * speed_likelihood
        


class StateVector:
    def __init__(self, Es, Is, Ic, P, S):
        self.Es = Es # {"go", "stop"} 
        self.Is = Is # {"go", "stop"}
        self.Ic = Ic # {"left", "straight", "right"}
        self.P = P   # (x, y, theta)
        self.S = S   # scalar


