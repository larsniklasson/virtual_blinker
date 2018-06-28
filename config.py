from utils.Intersection import Intersection

GEN_CONFIG = {
    "intersection": Intersection(),
    #plot,save,nr_cars done in sim.launch
}

def car(td, turn, sd, re):
    return {"travelling_direction": td, 
            "turn": turn, 
            "starting_distance": sd, 
            "use_riskestimation": re}

CARS = {
    0 : car("south", "straight", 65, True),
    1 : car("south", "left", 20, False),
    2 : car("south", "right", 45, True),
    3 : car("north", "left", 50, True),
    4 : car("north", "straight", 30, True),
    5 : car("east", "right", 50, True),
    6 : car("east", "left", 70, True),
    7 : car("west", "right", 100, True),
    8 : car("west", "left", 20, True),
    9 : car("west", "straight", 70, True)
}

SIM_CONFIG = {
    "xy_deviation" : 0.3,
    "theta_deviation" : 0.1,
    "speed_deviation": 0.2,
    "slowdown": 1.5,
    "rate": 10,
    "pid" : (0.4, 0.0, 0.05),
    "lookahead": 5,
    "carlength": 4,

    "discard_measurement_time": 0.3, #seconds
    "Es_threshold": 0.5,
    "risk_threshold": 0.3,
    "save_id" : 1

}

RISK_CONFIG = {
    "grant_threshold": 0.9


}
