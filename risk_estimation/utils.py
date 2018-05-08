import math
import numpy as np
#from particle_filter import state_vector
import particle_filter
import matplotlib.pyplot as plt
import matplotlib.patches as patchesk
import time

"""
vehicle pose contains x,y,theta of a vehicle. 
we use this function to project the said vehicle into a course path. 
for forway intersectoins, we have three courses, 
    1) turn left at the intersection
        in the ideal case, car should follow an arc starting from its right lane
        and ending in the right lane of the left road , this will be a one fourth of a circle.
    2) go straight ahead.
        no change in theta , just straight ahead. arc_project wont be useful in this situation
    3) turn right
        in here also, ideal path would be an arc starting from its right lane and
        ending in the right lane of the right road. 
we use arc_project to see where the vehicle should be in 1 and 3. 

for the purpose of this project, our intersections are right angles.
- assumes that the vehicle_pose is already inside the intersection

start_dir and end_dir are starting and ending  directions wrt the intersection
"""

def rotate_point(origin, point, angle):
    #adapted from https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin[0], origin[1]
    px, py = point[0], point[1]

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy
def pol2rect(r,theta):
    return (math.cos(theta)*r, math.sin(theta)*r)

def angle(p1,p2):
    """ gives angle formed by line segment p1 and p2. Angle is given with respect to p2. 

    Args:
        p1 (tuple of x,y,):  point p1
        p2 (tuple of x,y,):  point p2

    Returns:
        float. Angle in radians.

    Example:
        p1 = (0,0), p2 = (0,9), angle(p1,p2) gives 3pi/2 (270 degrees). angle(p2,p1) gives pi/2 (90 degrees)
    """

    point_wrt_p2 = (-p2[0] + p1[0], -p2[1] + p1[1])
    angle = 0
    angle = math.atan2(point_wrt_p2[1], point_wrt_p2[0])
    angle = angle + 2*math.pi if angle < 0 else angle
    """
    if (point_wrt_p2[0] == 0):
        if point_wrt_p2[1] >= 0: 
            angle = math.pi/2
        else:
            angle = 3* math.pi/2
    else:
        angle = math.atan(point_wrt_p2[1]/point_wrt_p2[0])
    """
    return angle

def pose_avg(p1,p2):
    #computes average of two poses:
    avg_x = 0.5 * (p1[0] + p2[0])
    avg_y = 0.5 * (p1[1] + p2[1])

    #average theta will be a bit different from normal average due to angle reset at theta = 0
    theta_1 = p1[2]
    theta_2 = p2[2]
    
    if ( (theta_1 >=0 and theta_1 <= math.pi/2) and (theta_2 >= 3*math.pi/2 and theta_2 <= 2*math.pi)  ) or \
       ( (theta_2 >=0 and theta_2 <= math.pi/2) and (theta_1 >= 3*math.pi/2 and theta_1 <= 2*math.pi)  ):
        #print("theta 1 = " + str(math.degrees(theta_1)))
        #print("theta 2 = " + str(math.degrees(theta_2)))
        avg_theta = (0.5 * (theta_1 + theta_2)) + math.pi
    else:
        avg_theta = (0.5 * (theta_1 + theta_2)) 
        

    return (avg_x,avg_y,avg_theta)




    

def pyth_distance(p1,p2):
    return math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )

def offset_relative(origin,vector):
    return (origin[0]+vector[0], origin[1] + vector[1])

def place_vehicle(intersection_center,lane_width,direction,distance_further):
    #returns the pose of a vehicle that is placed distance_further orthogonally from the intersection center,
    #and direction. vehicle is posed on the middle of right lane, facing towards intersection, 
    if (direction == 'east'):
        rotation_angle = 0
    if (direction == 'north'):
        rotation_angle = math.pi/2
    if (direction == 'west'):
        rotation_angle = math.pi
    if (direction == 'south'):
        rotation_angle = 3*math.pi/2
    
    point = offset_relative(intersection_center,(distance_further,lane_width/2))
    p = rotate_point(intersection_center,point,rotation_angle)
    return (p[0],p[1],rotation_angle+math.pi) # will be facing opposite direction where it is coming from

def arc_project (start_pose,end_pose,start_dir,end_dir,vehicle_pose):
    v_x,v_y,_ = vehicle_pose

    s_x,s_y,s_theta = start_pose
    e_x,e_y,e_theta = end_pose
    
    ideal_x = 0
    ideal_y = 0
    ideal_theta = 0

    if (start_dir == 'south' and end_dir == 'west'):
        radius = s_x - e_x #thought: could this be negative at all?
        angle = math.atan2(v_y,v_x)  # angle made by vechile and starting point
        ideal_x = radius * math.cos(angle)
        ideal_y = radius * math.sin(angle)
        ideal_theta = angle + math.pi/2
    
    if(start_dir == 'south' and end_dir == 'east'):
        radius = e_x - s_x
        angle = math.atan2 (v_y,v_x)
        ideal_x = radius * math.cos(angle)
        ideal_y = radius * math.cos(angle)
        ideal_theta = angle - math.pi/2


    return (ideal_x,ideal_y,ideal_theta)



    pass

def plot_particle_distribution(particles,weights):
    fig , ax = plt.subplots(nrows=3,ncols=1,figsize=(10,10))

    #es plot
    es_ = [x.Es for x in particles]
    blob = zip(es_,weights)
    #get weights of particles where es = stop
    stop_weights = sum([x[1] for x in filter(lambda x: x[0] == 0,blob)])


    #get weights of particles where es = go
    go_weights = sum([x[1] for x in filter(lambda x: x[0] == 1,blob)])

    ax[0].set_title("Es ditribution")
    ax[0].bar([0,1],[stop_weights,go_weights])


    #is plot
    is_ = [x.Is for x in particles]
    blob = zip(is_,weights)
    #get weights of particles where es = stop
    stop_weights = sum([x[1] for x in filter(lambda x: x[0] == 0,blob)])

    #get weights of particles where es = go
    go_weights = sum([x[1] for x in filter(lambda x: x[0] == 1,blob)])
    ax[1].set_title("Is ditribution")
    ax[1].bar([0,1],[stop_weights,go_weights])

    #ic plot
    #another idea to get most likely state: find most weighted variable from each variable
    #get Ic
    ic_ = [x.Ic for x in particles]
    blob = zip(ic_,weights)
    uturn_weights = sum([x[1] for x in filter(lambda x: x[0] == 0,blob)])
    left_turn_weights = sum([x[1] for x in filter(lambda x: x[0] == 1,blob)])
    straight_ahead_weights = sum([x[1] for x in filter(lambda x: x[0] == 2,blob)])
    right_turn_weights = sum([x[1] for x in filter(lambda x: x[0] == 3,blob)])
    ax[2].set_title("course")
    ax[2].bar([0,1,2,3], [uturn_weights,left_turn_weights,straight_ahead_weights,right_turn_weights])

    fig.savefig("./distro_plot_" +str(int((round(time.time()*1000)))),dpi=100)


    pass

def get_opposite_direction(dir):
    if (dir == 'north'):
        return 'south'
    elif (dir == 'east'):
        return 'west'
    elif (dir == 'south'):
        return 'north'
    else: # (dir == 'west'):
        return 'east'



def generate_inital_particles(inital_statevector,amount,pose_covariance, speed_deviation):
    #generate amount number of state vectors from initial distribution
    #since initial distribution is essentialy a delta function, (spike at x0, 0 elsewhere),
    #sample drawn from it matches that x0 exactly all the time. 
    #for this initial, i would like to generate half of particles have exact same vector as the intial parameters for x0
    #and the rest with varying information for state vectors
    #^ above may be wrong

    #generating new particles from the following method:
    particles = []


    for i in range(amount):
        #discrete var
        #for now this is uniform random

        Es_ = np.random.randint(0,2)
        Is_ = np.random.randint(0,2)
        Ic_ = np.random.randint(1,4)
        
        #in this setup we know the courses:

        #Ic_ = inital_statevector.Ic

        #continuous:
        P_ = np.random.multivariate_normal(inital_statevector.P,pose_covariance)
        S_ = np.random.normal(inital_statevector.S,speed_deviation)
        particles.append(particle_filter.state_vector(Es_,Is_,Ic_,P_,S_))
        #particles.append(deepcopy(inital_statevector))
    
    return particles