from copy import deepcopy
import numpy as np
from E_estimate import Es_estimate
from I_estimate import Is_estimate, Ic_estimate, choice
from PS_estimate import *
import math
import random


class ParticleFilter:
    def __init__(self,travelling_directions, intersection, n_particles, initial_measurement, pose_covariance, speed_deviation):
        
        self.weights = []
        
        self.n_particles = n_particles

        #travelling directions for all the vehicles
        self.travelling_directions  = travelling_directions

        self.intersection = intersection


        self.position_covariance =  pose_covariance[np.ix_([0,2],[0,2])]
        self.theta_deviation = pose_covariance[2][2]

        self.pose_covariance  = pose_covariance
        self.speed_deviation = speed_deviation

        #resampling threshold
        self.neff_threshold = self.n_particles / 2

        P = initial_measurement[:3]
        S = initial_measurement[-1]

        #place first gen particles around first measurement
        self.particles = generate_inital_particles(intersection, P, S, n_particles, self.pose_covariance, self.speed_deviation)
        self.weights = [1.0 / self.n_particles] * self.n_particles

        
        #is changed if intention overridden
        self.known_Is = None
        self.known_Ic = None

        #densities for Es, Is, Ic. Are updated after one time step. Most likely state is
        # derived from this. I saved this mainly to be able to use this in the plot
        self.Es_density = {"go": 0.5, "stop":0.5}
        self.Is_density = {"go": 0.5, "stop":0.5}
        turns = intersection.turns
        self.Ic_density = {t: 1.0/len(turns) for t in turns}

        #best P and S, also used when calculating most likely state
        self.best_P = P 
        self.best_S = S
    

    #override functions. Updates existing particles as well.#TODO unclear if this is better or not
    def setKnownIc(self, Ic):
        self.known_Ic = Ic
        for p in self.particles:
            p.Ic = Ic

    def setKnownIs(self, Is):
        self.known_Is = Is
        for p in self.particles:
            p.Is = Is
    
    def removeKnownIs(self, Is):
        self.known_Is = None

    def removeKnownIc(self, Ic):
        self.known_Ic = None

    #method used in resampling step 
    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    # find density of Es, Is and Ic.
    def updateDensities(self):
        
        #zero out these
        for k in self.Es_density:
            self.Es_density[k] = 0
            self.Is_density[k] = 0
        for t in self.Ic_density:
            self.Ic_density[t] = 0
        
        bx = 0
        by = 0
        btheta = 0
        bs = 0
        for p, w in zip(self.particles, self.weights):
            self.Es_density[p.Es] += w
            self.Is_density[p.Is] += w
            self.Ic_density[p.Ic] += w
            bx += p.P[0] * w
            by += p.P[1] * w
            btheta += p.P[2] *w
            bs += p.S*w

        
        self.best_P = bx, by, btheta
        self.best_S = bs

        #self.best_P = np.sum([np.array(p.P)*w for p,w in zip(self.particles, self.weights)], axis=0)
        #self.best_S = np.sum([p.S*w for p,w in zip(self.particles, self.weights)], axis=0)

    
    def get_most_likely_state(self):

        return self.Ic_density, self.best_P, self.best_S
        
    
    def step_time(self, id, measurement_vector, most_likely_states, interval, ttc_hli):
        #resample particles by their weight:W
        #sources obtained: 
        # for neff comparision: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
        # This is a sampling importance resampling. 
        # http://people.math.aau.dk/~kkb/Undervisning/Bayes14/sorenh/docs/sampling-notes.pdf
        
        if self.neff(self.weights) < self.neff_threshold:
            #indices of particles to resample
            resampled = np.random.choice(self.particles, len(self.particles), p=self.weights)

            pc = {}
            for p in resampled:
                if p in pc:
                    pc[p] += 1
                else:
                    pc[p] = 1

            
            e = {}
            ps = {}
            
            for p in pc:
                _,es = Es_estimate(id, p.Ic, p.P, p.S, self.travelling_directions, self.intersection, most_likely_states, ttc_hli)
                e[p] = es
                if self.known_Is:
                    Is_list = [self.known_Is]
                else:
                    Is_list = ["go", "stop"]
                if self.known_Ic:
                    Ic_list = [self.known_Ic]
                else:
                    Ic_list = self.intersection.turns

                for i in Is_list:
                    for c in Ic_list:
                        ps[i,c] = PS_estimate(p.P[0], p.P[1], p.P[2], p.S, i, c, self.travelling_directions[id], self.intersection, interval, self.pose_covariance, self.speed_deviation, flag=True)
            

            new_particles = []
            for p,v in pc.iteritems():
                for _ in range(v):
                    
                    
                    new_Es = "go" if random.random() <= e[p] else "stop"
                    
                    

                    #TODO this is a bit ugly
                    if self.known_Is:
                        new_Is = self.known_Is
                    else:
                        new_Is = Is_estimate(p.Is, new_Es)

                    if self.known_Ic:
                        new_Ic = self.known_Ic
                    else:
                        new_Ic = Ic_estimate(p.Ic, self.intersection.turns)
                    
                    
                    new_P, new_S = sample(*ps[new_Is, new_Ic], pose_covariance=self.pose_covariance, speed_deviation=self.speed_deviation)    


                    new_particles.append(StateVector(new_Es ,new_Is, new_Ic, new_P, new_S))








            


            #resampled = np.random.choice(self.particles, self.n_particles, p = self.weights)
            #self.particles = resampled

        else:
            new_particles = []
            for p in self.particles:
                #project new state

                new_Es,_ = Es_estimate(id, p.Ic, p.P, p.S, self.travelling_directions, self.intersection, most_likely_states, ttc_hli)

                #TODO this is a bit ugly
                if self.known_Is:
                    new_Is = self.known_Is
                else:
                    new_Is = Is_estimate(p.Is, new_Es)

                if self.known_Ic:
                    new_Ic = self.known_Ic
                else:
                    new_Ic = Ic_estimate(p.Ic, self.intersection.turns)

                new_P, new_S = PS_estimate(p.P[0], p.P[1], p.P[2],p.S, new_Is, new_Ic, self.travelling_directions[id], self.intersection, interval, self.pose_covariance, self.speed_deviation)
                
                new_particles.append(StateVector(new_Es ,new_Is, new_Ic, new_P, new_S))
            
        new_weights = [self.likelihood(p, measurement_vector) for p in new_particles]


        #normalize weights
        w_sum = float(sum(new_weights))
        new_weights = [w/w_sum for w in new_weights]

        #update global lists
        self.weights = new_weights
        self.particles = new_particles

        #update the densities for the state variables
        self.updateDensities()
        
        

    
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
        mean_x, mean_y = p.P[:2]
        mean_theta = p.P[-1]
        mean_speed = p.S

        x,y = measurement_vector[:2]
        theta = measurement_vector[2]
        speed = measurement_vector[3]

        a = self.position_covariance[0][0]
        x_likelihood = normpdf(x, mean_x, a)
        y_likelihood = normpdf(y, mean_y, a)
        position_likelihood = x_likelihood * y_likelihood
        #position_likelihood = multivariate_normal.pdf(xy,mean = mean_xy, cov=self.position_covariance)

        
        upper_bound = mean_theta + math.pi
        lower_bound = mean_theta - math.pi

        while (theta < lower_bound or theta > upper_bound): #TODO make this look better, surely should not be necessary to have a while here?
            if theta < lower_bound:
                theta += math.pi*2
            if theta > upper_bound:
                theta -= math.pi*2


        #theta_likelihood = norm.pdf(theta, loc=mean_theta, scale=self.theta_deviation)
        theta_likelihood = normpdf(theta, mean_theta, self.theta_deviation)

        #speed_likelihood = norm.pdf(speed, loc=mean_speed, scale=self.speed_deviation)
        speed_likelihood = normpdf(speed, mean_speed, self.speed_deviation)

        
        pose_likelihood = position_likelihood * theta_likelihood # position and angle are independent so this multiplication is okay

        return pose_likelihood * speed_likelihood


def normpdf(x, mu, sigma):
    u = float((x-mu) / abs(sigma))
    y = math.exp(-u*u/2) / (math.sqrt(2*math.pi) * abs(sigma))
    return y
        

def generate_inital_particles(intersection, initial_pose, initial_speed, nr_particles, pose_covariance, speed_deviation):
    particles = []

    Ic_density = [1.0/len(intersection.turns)]*len(intersection.turns)
    for i in range(nr_particles):
        
        x,y,theta = initial_pose
        speed = initial_speed
        xy_cov = pose_covariance[0][0]
        theta_cov = pose_covariance[2][2]

        x_estimate = np.random.normal(x, xy_cov)
        y_estimate = np.random.normal(y, xy_cov)
        theta_estimate = np.random.normal(theta, theta_cov)
        P = x_estimate, y_estimate, theta_estimate

        #p_estimate = np.random.multivariate_normal((xnew, ynew, thetanew), pose_covariance)
        S = np.random.normal(speed, speed_deviation)
        
        Es = "go" if random.random() <= 0.5 else "stop"
        Is = "go" if random.random() <= 0.5 else "stop"
        Ic = choice(intersection.turns, Ic_density)
    
        P = np.random.multivariate_normal(initial_pose, pose_covariance)
        S = np.random.normal(initial_speed, speed_deviation)

        particles.append(StateVector(Es,Is,Ic,P,S))
    return particles


class StateVector:
    def __init__(self, Es, Is, Ic, P, S):
        self.Es = Es # {"go", "stop"} 
        self.Is = Is # {"go", "stop"}
        self.Ic = Ic # {"left", "straight", "right"}
        self.P = P   # (x, y, theta)
        self.S = S   # scalar

