from utils.Intersection import Intersection

GEN_CONFIG = {
    "wipe_plot_dir": True,
    "intersection": Intersection(),

    #plot,save,nr_cars done in sim.launch


}

def car(td, turn, sd, re):
    return {"travelling_direction": td, 
            "turn": turn, 
            "starting_distance": sd, 
            "use_riskestimation": re}

CARS = {
    0 : car("east", "straight", 40, False),
    1 : car("south", "straight", 40, True),
    2 : car("east", "right", 50, True),
    3 : car("west", "left", 50, True)

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
    "save_id" : 1

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
