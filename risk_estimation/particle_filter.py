from copy import deepcopy
import numpy as np
from E_estimate import Es_estimate
from I_estimate import Is_estimate, Ic_estimate, choice
from PS_estimate import *
import math
import random


class ParticleFilter:
    def __init__(self,travelling_directions, intersection, n_particles, initial_measurement, deviations):
        
        self.weights = []
        
        self.n_particles = n_particles

        #travelling directions for all the vehicles
        self.travelling_directions  = travelling_directions

        self.intersection = intersection
        self.deviations = deviations
        self.xy_deviation, self.theta_deviation, self.speed_deviation = deviations

        #resampling threshold
        self.neff_threshold = self.n_particles / 2

        PS = initial_measurement

        #place first gen particles around first measurement
        self.particles = self.generate_inital_particles(PS)
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
        self.best_PS = PS
    

    #override functions. Updates existing particles as well.#TODO unclear if this is better or not
    def setKnownIc(self, Ic):
        self.known_Ic = Ic
        #for p in self.particles:
        #    p.Ic = Ic

    def setKnownIs(self, Is):
        self.known_Is = Is
        #for p in self.particles:
        #    p.Is = Is
    
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
        
        self.best_PS = np.array((0.0, 0.0, 0.0, 0.0))
        for p, w in zip(self.particles, self.weights):
            self.best_PS += p.PS * w
            self.Es_density[p.Es] += w
            self.Is_density[p.Is] += w
            self.Ic_density[p.Ic] += w

    
    def get_most_likely_state(self):

        return self.Ic_density, self.best_PS
        
    
    def step_time(self, id, measurement_vector, most_likely_states, interval, precalculation):

        if self.neff(self.weights) < self.neff_threshold:
            #resampling
            #To avoid recalculating for the same particle we save the Es density and the 
            # predicted next state (for all combinations of Is/Ic) for that particle
            # later the exact same particle that was resampled more than one time can just lookup these values

            #indices of particles to resample
            indices = np.random.choice(range(len(self.particles)), len(self.particles), p=self.weights)
            blob = [(self.particles[Is], self.weights[Is]) for Is in indices]
            resampled_particles, old_weights = zip(*blob)

            particle_count = {}
            for p in resampled_particles:
                if p in particle_count:
                    particle_count[p] += 1
                else:
                    particle_count[p] = 1

            
            Es_dict = {}
            PS_dict = {}
            
            for p in particle_count:
                _, es_density = Es_estimate(id, p.Ic, p.PS, self.travelling_directions, 
                                            self.intersection, most_likely_states, precalculation)
                Es_dict[p] = es_density

                Is_list = [self.known_Is] if self.known_Is else ["go", "stop"]
                Ic_list = [self.known_Ic] if self.known_Ic else self.intersection.turns

                for Is in Is_list:
                    for Ic in Ic_list:
                        PS_dict[p,Is,Ic] = \
                                PS_estimate(p.PS, Is, Ic, 
                                            self.travelling_directions[id], 
                                            self.intersection, interval, self.deviations, no_sample=True)
            

            new_particles = []
            for p,v in particle_count.iteritems():
                for _ in range(v):
                    
                    new_Es = "go" if random.random() <= Es_dict[p] else "stop"
                        
                    new_Is = self.known_Is if self.known_Is else Is_estimate(p.Is, new_Es)
                    new_Ic = self.known_Ic if self.known_Ic else Ic_estimate(p.Ic, self.intersection.turns)
                    
                    new_PS = sample(*PS_dict[p,new_Is, new_Ic])

                    new_particles.append(StateVector(new_Es ,new_Is, new_Ic, new_PS))


        else:
            #no resampling
            old_weights = self.weights
            new_particles = []
            for p in self.particles:
                #project new state

                new_Es,_ = Es_estimate(id, p.Ic, p.PS, self.travelling_directions, self.intersection, most_likely_states, precalculation)

                new_Is = self.known_Is if self.known_Is else Is_estimate(p.Is, new_Es)
                new_Ic = self.known_Ic if self.known_Ic else Ic_estimate(p.Ic, self.intersection.turns)

                new_PS = PS_estimate(p.PS, new_Is, new_Ic, self.travelling_directions[id], self.intersection, interval, self.deviations)
                
                new_particles.append(StateVector(new_Es ,new_Is, new_Ic, new_PS))
            

        new_weights = [self.likelihood(p.PS, measurement_vector)*w for p,w in zip(new_particles, old_weights)]


        #normalize weights
        w_sum = float(sum(new_weights))
        new_weights = [w/w_sum for w in new_weights]

        #update global lists
        self.weights = new_weights
        self.particles = new_particles

        #update the densities for the state variables
        self.updateDensities()
        
        

    
    def likelihood(self, PS, measurement_vector):
        (mean_x, mean_y, mean_theta, mean_speed) = PS
        x,y,theta,speed = measurement_vector

        x_likelihood = normpdf(x, mean_x, self.xy_deviation)
        y_likelihood = normpdf(y, mean_y, self.xy_deviation)
        
        upper_bound = mean_theta + math.pi
        lower_bound = mean_theta - math.pi

        while (theta < lower_bound or theta > upper_bound): 
            if theta < lower_bound:
                theta += math.pi*2
            if theta > upper_bound:
                theta -= math.pi*2


        theta_likelihood = normpdf(theta, mean_theta, self.theta_deviation)

        speed_likelihood = normpdf(speed, mean_speed, self.speed_deviation)

        
        return x_likelihood * y_likelihood * theta_likelihood * speed_likelihood


    def generate_inital_particles(self, initial_ps):
        particles = []
        nr_turns = len(self.intersection.turns)
        Ic_density = [1.0/nr_turns]*nr_turns
        for _ in range(self.n_particles):
            
            x,y,theta,speed = initial_ps
            x_estimate = np.random.normal(x, self.xy_deviation)
            y_estimate = np.random.normal(y, self.xy_deviation)
            theta_estimate = np.random.normal(theta, self.theta_deviation)
            s_estimate = np.random.normal(speed, self.speed_deviation)

            PS = np.array((x_estimate, y_estimate, theta_estimate, s_estimate))

            
            Es = "go" if random.random() <= 0.5 else "stop"
            Is = "go" if random.random() <= 0.5 else "stop"
            Ic = choice(self.intersection.turns, Ic_density)
        
            particles.append(StateVector(Es,Is,Ic,PS))
        return particles



def normpdf(x, mu, sigma):
    u = float((x-mu) / abs(sigma))
    y = math.exp(-u*u/2) / (math.sqrt(2*math.pi) * abs(sigma))
    return y
        


class StateVector:
    def __init__(self, Es, Is, Ic, PS):
        self.Es = Es # {"go", "stop"} 
        self.Is = Is # {"go", "stop"}
        self.Ic = Ic # {"left", "straight", "right"}
        self.PS = PS   # (x, y, theta, speed)

