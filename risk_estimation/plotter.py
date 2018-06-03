
import sys
sys.path.append("..")
from utils.Intersection import *
import numpy as numpy

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

import math
import os

def draw_arrow(fig,ax,vehicle_pose,length):
    x = vehicle_pose[0]
    y = vehicle_pose[1]
    theta = vehicle_pose[2]
    dx = length*math.cos(theta)
    dy = length*math.sin(theta)
    ax.arrow(x,y,dx,dy,head_width=2.15, head_length=2.1,fc='green', ec='green')



def plot_particles(p_filters, measurement_vector, t, plot_folder):
    caption_text = ""
    
    fig , ax = plt.subplots(figsize=(15,15))
    
    ax.set_title("t = " + str(t))
    
    ax.spines['left'].set_position('center')
    ax.spines['bottom'].set_position('center')

    xlim = (-80, 80)
    ylim = (-80, 80)

    plt.xlim(*xlim)
    plt.ylim(*ylim)
    plt.autoscale(False)
    ax.add_patch(
        patches.Rectangle(
            (-7.5,-7.5),  
            7.5*2,   #width
            7.5*2,   #height
            fill= False
        )
    )
    
    for i, pfilter in enumerate(p_filters):
        x = [p.PS[0] for p in pfilter.particles]
        y = [p.PS[1] for p in pfilter.particles]
        best_PS = pfilter.best_PS

        caption_Es = {k:round(v,2) for k, v in pfilter.Es_density.iteritems()}
        caption_Is = {k:round(v,2) for k, v in pfilter.Is_density.iteritems()}
        caption_Ic = {k:round(v,2) for k, v in pfilter.Ic_density.iteritems()}
        caption_P = [round(field,2) for field in best_PS[:3]]
        caption_S = round(best_PS[-1],2)

        vehicle_text  = "Id {0}:\n Es = {1}\n Is = {2}\n Ic = {3}\n Best Pose = {4} \n Best speed = {5} \n\n"
        caption_text  = caption_text + vehicle_text.format( \
                                            i, \
                                            caption_Es,\
                                            caption_Is,\
                                            caption_Ic,\
                                            caption_P,\
                                            caption_S)
        
        ax.scatter(x, y, s=1)
        ax.scatter(best_PS[0], best_PS[1], c='y')
        for m in measurement_vector:

            ax.scatter(m[0],m[1], c='r')
        
        draw_arrow(fig, ax, best_PS[:3], 4)
    

    ax.text(xlim[0],ylim[0] ,caption_text,fontsize=15)

    filename = "%.2f" % t + ".png"
    fp = plot_folder + "/" + filename
    fig.savefig(fp,dpi=100)
    plt.close(fig)
    
    
    