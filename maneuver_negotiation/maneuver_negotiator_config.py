GENERAL_OPTIONS = {
    'zookeeper-server' : '127.0.0.1:2181',
    'TM' : 10 ,#Come up with better values
    'TD' : 2,
    'TMan' : 20,
    'CommRadius': 200,
    'TA': 0.5
}

UDP_COMMUNICATION_OPTIONS = {
    #key = agent id,
    #value = [ip address, port]
    0: ['127.0.0.1', 9000],
    1: ['127.0.0.1', 9001],
    2: ['127.0.0.2', 9002],
    3: ['127.0.0.3', 9003],
    4: ['127.0.0.4', 9004],
    5: ['127.0.0.5', 9005]
}

ROS_COMMUNICATION_OPTIONS = {

    'maneuver-negotiation-topics' : {
    # key = agent id
    # value = [ros topic to subscribe to get maneuver messages from other cars,
    #          ros topic to subscribe to listen and initiate a maneuver ]

    0: ['/car0_mn','car0_control'],
    1: ['/car1_mn','car1_control'],
    2: ['/car2_mn','car2_control'],
    3: ['/car3_mn','car3_control'],
    4: ['/car4_mn','car4_control'],
    'choose-leader-topic' : '/chooseleader'
    },

    'car-state-topic' : {
        0: '/car_state0',
        1: '/car_state1',
        2: '/car_state2',
        3: '/car_state3',
        4: '/car_state4'

    }
}