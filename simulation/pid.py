#!/usr/bin/python


import time

class PID:

    def __init__(self, P, I, D, windup_guard):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.ITerm = 0

        self.last_time = time.time()
        self.windup_guard = windup_guard

        self.last_error = 0.0
        self.clear()

    def clear(self):
        self.ITerm = 0.0
        self.last_time = time.time()
        

    def update(self, error):
        
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        delta_error = error - self.last_error


        PTerm = self.Kp * error
        self.ITerm += error * delta_time

        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard

        DTerm = 0.0
        if delta_time > 0:
            DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = current_time
        self.last_error = error

        return PTerm + (self.Ki * self.ITerm) + (self.Kd * DTerm)

