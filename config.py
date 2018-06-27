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
    0 : car("north", "straight", 0, True),
    1 : car("west", "straight", 20, True),
    2 : car("east", "left", 40, True),
    3 : car("north", "left", 50, True),
    4 : car("south", "straight", 60, True),
    5 : car("east", "right", 0, True),
    6 : car("north", "right", 90, True),
    7 : car("west", "right", 100, True),
    8 : car("north", "right", 0, True),
    9 : car("east", "straight", 70, True)
}

SIM_CONFIG = {
    "xy_deviation" : 0.3,
    "theta_deviation" : 0.1,
    "speed_deviation": 0.2,
    "slowdown": 1,
    "rate": 20,
    "pid" : (0.4, 0.0, 0.05),
    "lookahead": 5,
    "carlength": 4,

    "discard_measurement_time": 0.2, #seconds
    "Es_threshold": 0.5,
    "risk_threshold": 0.8,
    "save_id" : 1

}

RISK_CONFIG = {
    "grant_threshold": 0.5


}
