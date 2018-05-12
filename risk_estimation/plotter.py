
import numpy as numpy

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import Intersection
import math
import config

def draw_arrow(fig,ax,vehicle_pose,length):
    x = vehicle_pose[0]
    y = vehicle_pose[1]
    theta = vehicle_pose[2]
    dx = length*math.cos(theta)
    dy = length*math.sin(theta)
    ax.arrow(x,y,dx,dy,head_width=1.15, head_length=1.1,fc='k', ec='k')



def plot_particles(particle_filter_list, measurement_vector, t, riskdict, plot_folder):
    caption_text = ""
    fig , ax = plt.subplots(figsize=(10,10))
    ax.set_title("t = "+str(t)  + "\n" +str(riskdict) + "\n")
    ax.spines['left'].set_position('center')
    ax.spines['bottom'].set_position('center')
    axbound = 90
    plt.xlim(-axbound, axbound)
    plt.ylim(-axbound, axbound)
    plt.autoscale(False)
    ax.add_patch(
        patches.Rectangle(
            (-7.5,-7.5),  #origin
            7.5*2,   #width
            7.5*2,   #height
            fill= False
        )
    )
    
    for i,pfilter in enumerate(particle_filter_list):
        poses = [x.P for x in pfilter.particles]
        vehicle_state = pfilter.get_most_likely_state()
        x = [q[0] for q in poses]
        y = [q[1] for q in poses]
        #theta = [q[2] for q in poses]
        vehicle_text  = "Vehicle {0}:\n Expectation = {1}\n Intention = {2}\n Intended course = {3}\nDirection = {4} deg \n\n"
        caption_text  = caption_text + vehicle_text.format(i,vehicle_state.Es,\
                        vehicle_state.Is,\
                        vehicle_state.Ic,\
                        round(math.degrees(vehicle_state.P[2]),1))
        
        ax.scatter(x,y,c=str(i),s=1)
        ax.scatter(vehicle_state.P[0],vehicle_state.P[1],c='y')
        ax.scatter(measurement_vector[0][0],measurement_vector[0][1],c='r')
        ax.scatter(measurement_vector[1][0],measurement_vector[1][1],c='r')
        draw_arrow(fig,ax, vehicle_state.P,2)
    
    
    #ax.scatter(measurement_vector[0][0],measurement_vector[0][1],c='r')
    #ax.text(measurement_vector[0][0],measurement_vector[0][1] - 20,"vehicle 0",fontsize=12)
    #ax.scatter(measurement_vector[1][0],measurement_vector[1][1],c='r')
    #ax.text(measurement_vector[1][0] + 20,measurement_vector[1][1],"vehicle 1",fontsize=12)

    ax.text(-axbound,-axbound,caption_text,fontsize=15)

    

    fig.savefig(plot_folder + "/plot_" +str(int((round(time.time()*1000)))),dpi=100)
    plt.close(fig)
    
    
    