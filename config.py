from utils.Intersection import Intersection

GEN_CONFIG = {
    "wipe_plot_dir": True,
    "intersection": Intersection(),
    "save_directory": "../experiment_results/blue_accelerate_green_constant/acceleration/1/",
    "simulation_end_time": 10.0

    #plot,save,nr_cars done in sim.launch


}

def car(td, turn, sd, re, ki,follow_expectation,emergency_break):
    return {"travelling_direction": td, 
            "turn": turn, 
            "starting_distance": sd, 
            "use_riskestimation": re,
            "use_known_I": ki,
            "follow_expectation":follow_expectation,
            "emergency_break":emergency_break}


CARS = {
    0 : car("north", "left", 50,True, False,False,False),
    1 : car("south", "straight", 50,True, True,True,True),
    2 : car("east", "right", 50, True, True,True,True),
    3 : car("west", "left", 50, True, True,True,True)

}

SIM_CONFIG = {
    "xy_deviation" : 0.3,
    "theta_deviation" : 0.1,
    "speed_deviation": 0.2,
    "slowdown": 1.5,
    "rate": 15,
    "pid" : (0.4, 0.0, 0.05),
    "lookahead": 5,
    "carlength": 4,

    "total_nr_particles": 2500,
    "discard_measurement_time": 0.15, #seconds
    "Es_threshold": 0.5,
    "risk_threshold": 0.8,
    "save_id" : 0

}

RISK_CONFIG = {
    "prediction_dev_coeff": 2,
    "gap_model": {
        "L": 1.0,
        "x0": 6.1,
        "k": 1,

        "x0_2": 1.5,
        "k2": 3
    },
    "Is_comply": 0.75,
    "Ic_same": 0.5,
    "grant_threshold": 0.5


}
