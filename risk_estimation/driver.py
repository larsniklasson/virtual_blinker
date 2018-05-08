from collections import OrderedDict
from particle_filter import particle_filter,state_vector
import plotter
import config
import csv
import time
#this is the master python file that combines everything.

class risk_estimator:
    # assumptions:
    # never initialize the class with vechiles already inside the intersection
    # ie. vehicles are either approaching or leaving intersection

    def __init__(self, n_particles, interval,intersection_info,initial_measurements,pose_covariance,speed_deviation,initial_courses):
        #initial_measurement will be a list of (x,y,theta,speed)

        self.n_vehicles = len(initial_measurements)
        self.n_particles = n_particles
        self.intersection = intersection_info
        self.ved = [] # vehicle entering directions, wrt intersection
        self.interval = interval
        self.pose_covariance = pose_covariance
        self.speed_deviation = speed_deviation

        self.p_filterset = OrderedDict()

        #last timestamp received. This is useful when interval varies from 
        #instant to instant. -1 means we have not received a measurement yet
        self.last_t = -1

        # this class should inform each vehicles particle filter if the entering direction
        # is changed (eg when it has left intersection, turned around and entered from this
        # new direction)
        # shall we keep the ved tracking here?

        # populate ved, particle filter for each vehicle
        # print(initial_measurements)
        for vehicle_number,vehicle_measurement in initial_measurements.items():
            ans =  intersection_info.find_vehicle_pos(vehicle_measurement[:3])
            #print(str(ans))
            if ans == True:
                print("lies inside")
                pass
                # the vehicle lies inside intersection. whoops
            else:
                self.ved.append(ans[0])
                self.p_filterset[vehicle_number] = particle_filter(ans[0],intersection_info, n_particles,initial_courses[vehicle_number],vehicle_measurement,self.interval,self.pose_covariance,self.speed_deviation)

        if(config.GENERAL_OPTIONS['output-csv']):
            self.csvfile = open(config.GENERAL_OPTIONS['output-csv-folder'] + 'risks_output_' + str(int((round(time.time()*1000)))) +'.csv','w')
            self.csvwriter = csv.writer(self.csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            header = [("v" + str(x[0])) for x in initial_measurements.items()]
            header.insert(0,"time")
            self.csvwriter.writerow(header)

        #initial_measurements will be a dict with key = id, value = (x,y,theta,speed)

    def update_state(self,t,measurement_dict):
        #for each vehicle, we get the most likely state which is in using
        #particle filter
        courses = []
        poses = []
        speeds = []
        for k,pfilter in self.p_filterset.items():
            state = pfilter.get_most_likely_state()
            #print("vehicle " + str(k) + " Ic = " + str(state.Ic) + " Es = " + str(state.Es) + " Is = " + str(state.Is))
            #only consider vehicles that are not facing away from intersection
            #if not (self.intersection.is_vehicle_facing_away(state.P)):
            courses.append(state.Ic)
            poses.append(state.P)
            speeds.append(state.S)
        
        # if t is 0, it implies it just got started.
        # we ignore measurement at t = 0 for update step as we have
        # already hardcoded the initial state vectors of particles in the init step.
        if t < 0.001:
            self.last_t = 0.0
            return

        interval = t - self.last_t
        for k,pfilter in self.p_filterset.items():
            pfilter.step_time(measurement_dict[k],courses,poses,speeds,self.ved,t,interval)
        
        self.last_t = t
        
        risk = self.get_risk()
        if (config.GENERAL_OPTIONS['plot-particles']):
            plotter.plot_particles(self.p_filterset.values(),measurement_dict,t,risk)
        if (config.GENERAL_OPTIONS['output-csv']):
            risks = [x[1] for x in risk.items()]
            risks.insert(0,t)
            self.csvwriter.writerow(risks)

    

    def get_expectation_Es(self):
        #compute expectation of Es:
        expectations = OrderedDict()

        for k, pfilter in self.p_filterset.items():

            expectation = 0.
            for i,s in enumerate(pfilter.posterior_particles):
                weight = pfilter.posterior_weights[i]
                expectation = expectation + (s.Es*weight)



            expectations[k] = round(expectation,3)
            
        return expectations



    def get_risk(self):
        #risk calculation according to fench paper where:
        # in section 3.3 risk is defined from probability where:
        # P([Is != Es] | Z_0:t)
        # output is a dictionary where key = vehicle number, value = risk probability 
        # between 0 and 1 where 1 is highest risk
        """
        the particle filter estimates the posterior denstiy of the state given the 
        measurements. their weights are considered the probability distribution.
        in this case, P(Es = 0)  for a given vehicle at time t can be seen as the
        sum of the weights of all the particles where the state is Es =0
        for Is also, this can be reasoned out. (double check this)
        
        Mismatching Es and Is can be considered as 
        P( Es = 1 and Is = 0) OR P( Es = 0 and Is = 1)
        OR can be used for summing up two probabilities.

        So in this method we add up two set of weights:
        first set:
        weights of particles where Es = 1 and Is = 0
        
        second set:
        weights of particles where Es = 0 and Is = 1

        if we add these two, i think we got the risk.  the threshold for this sum is
            0.3 as defined in 5.1
        """

        riskset = OrderedDict()
        for k, pfilter in self.p_filterset.items():

            risk = 0.
            for i,s in enumerate(pfilter.posterior_particles):
                if ((s.Es == 0 and s.Is == 1) or (s.Es == 1 and s.Is == 0)):
                    risk = risk + pfilter.posterior_weights[i]
            

            riskset[k] = round(risk,3)
            
        return riskset
