NETWORK_OPTIONS = {
    'network-prefix' : '127.0.0.',
    'port-start' : 9000,
    'zookeeper-server' : '127.0.0.1:2181'
}
ROS_OPTIONS = {
    'measurement-topic' : '/measurements',
    'choose-leader-topic' : '/chooseleader'
}
GENERAL_OPTIONS = {
    #path were the python files for risk estimator lies.
    #maneuver negotiation needs this path because it also relies on 
    #Intersection datastructure to get the type of intersection 
    #maneuver negotiation is dealing with. 
    #this path will be appended as a library dependency lookup folder.
    #so that importing Intersection.py works
    'risk-estimator-path' : '/home/gazebo-laptop/testbed/src/risk_estimation/src/scripts/'
}