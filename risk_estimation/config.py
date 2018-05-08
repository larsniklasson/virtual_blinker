"""
This file defines some of the constants that are used in the french paper
Also stores some of the settings
"""
import numpy as np

GENERAL_OPTIONS = {
    #plot particle filter visualization 
    'plot-particles' : True,

    #folder to put all the successive images of each time interval
    'plot-folder' : './scenarios/leftturn/plots2',

    #output risk calculation as a csv
    'output-csv' : False,

    #folder for that
    'output-csv-folder': './scenarios/scenario4/',

    #input file
    'input-csv-file' : './scenarios/leftturn/go2.csv',
}

pose_cov = np.eye(3)*0.3
pose_cov[2][2] = 0.1

PARAMETERS = {


    # deviation of speed from one time instant to another. 
    # this deviation is low because i feel it is unlikely to change speeds so
    # suddenly within a short interval which we are sampling measurements

    'speed-deviation' : 1.2, 

    # pose covariance from one time instant to another. 
    # this also would be small, but larger than speed deviation because 
    # inside intersection we make turns and if speed is high, variance will be 
    # higher.
    'pose-covariance' : pose_cov,



    # how much measured speed would vary from actual speed, this deviation is
    # different from speed-deviation above because this depends on instrument which we
    # are measuring speed
    'measurement-speed-deviation' : 1.5,


    # Same, but for pose, 
    'measurement-pose-covariance' : 1.5,

}

#particle filter options
PF_OPTIONS = {
    #1 - exemplar path when outside intersection is a line towards intersection,
    #    independent of the intended course.
    #2 - exemplar path will be guided by the intended course. 
    'P_model' : 1

}