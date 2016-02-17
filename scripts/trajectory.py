#!/usr/bin/env python

# Imports
import time
import RPi.GPIO as GPIO

# Functions
def soft_move(self):
    """
    Creates a list of time values to use as PWM to move the platform
    according to multiple input parameters.
    Input:  motor_control node object
    Output: The time array associated to frequencies that form a
            trapeze (ramp up, plateau, ramp down).
    """

    # Return nothing if any argument is invalid
    used_params = [self.nb_pulse, self.f_max, self.f_min, self.plat_size, self.max_slope]
    for param in used_params:
        if param <= 0:
            print("Error - soft_move invalid argument : {0} <= 0".format(param))
            return None

    # Calculate center index
    center_ind = round(self.nb_pulse/2.)

    # Calculate beginning of plateau index
    plat_ind = round(center_ind-round(self.plat_size*self.nb_pulse)/2)

    # If no ramp, return slowest movement
    if plat_ind < 1:
        print("plat_ind < 1")
        dt = [0.5/self.f_min]*self.nb_pulse
        return(dt)

    # Calculate frequency slope
    slope = min((self.f_max-self.f_min)/(plat_ind), self.max_slope)

    # Generate instantaneous frequency points (trapeze)
    dF = [slope*i+self.f_min for i in range(int(plat_ind)+1)]
    dF = dF + [dF[-1]]*int(center_ind-plat_ind-1)
    buf = [dF[-1]] if self.nb_pulse&1 else []  # Account for even/odd number of points
    dF = dF[:int(center_ind)]
    dF = dF + buf + dF[::-1]

    # Time of half period
    dt = [0.5/f for f in dF]

    return dt

def axis_move(self):
    """
    Generates a clock signal on the GPIO pins of the motor_control node object.
    """

    dt = soft_move(self)

    GPIO.output(self.enable_pin, GPIO.LOW)
    time.sleep(0.01)

    for x in range(self.nb_pulse):
        time_begin = time.clock()
        GPIO.output(self.control_pin, GPIO.HIGH)

        while(time.clock() - time_begin < dt[x]):
            # Can be improved with interrupts
            pass

        time_begin = time.clock()

        GPIO.output(self.control_pin, GPIO.LOW)

        while(time.clock() - time_begin < dt[x]):
            # Can be improved with interrupts
            pass

    GPIO.output(self.enable_pin, GPIO.HIGH)


def pos_move(self):
    # Direction
    if self.delta < 0:
        self.direction = 0
        GPIO.output(self.dir_pin, self.direction)
    else:
        self.direction = 1
        GPIO.output(self.dir_pin, self.direction)

    # Movement
    nb_pulse_float = abs(self.delta)/self.pulse
    self.nb_pulse = int(round(nb_pulse_float))
    self.error_pulse += nb_pulse_float - self.nb_pulse

    # Adjust values if error is greater than 1 pulse
    if self.error_pulse < -1:
        self.error_pulse += 1
        self.nb_pulse -= 1
    elif self.error_pulse > 1:
        self.error_pulse -= 1
        self.nb_pulse += 1

    if self.nb_pulse != 0:
        axis_move(self)


def vel_move(self):
    self.vel_flag = 0

    # Do not move if velocity is 0
    if self.vel == 0:
        return

    # Direction
    if self.vel < 0:
        self.direction = 0
    else:
        self.direction = 1
    GPIO.output(self.dir_pin, self.direction)

    # Movement
    GPIO.output(self.enable_pin, GPIO.LOW)
    time.sleep(0.01)

    # Frequency boundaries
    self.vel = abs(self.vel)
    if self.vel < self.f_min:
        self.vel = self.f_min
    elif self.vel > self.f_max:
        self.vel = self.f_max

    # Half period
    dt = 0.5/self.vel
    vel_ini = self.vel

    while self.vel == vel_ini:
        time_begin = time.clock()
        GPIO.output(self.control_pin, GPIO.HIGH)

        while(time.clock() - time_begin < dt):
            # Can be improved with interrupts
            pass

        time_begin = time.clock()
        GPIO.output(self.control_pin, GPIO.LOW)

        while(time.clock() - time_begin < dt):
            # Can be improved with interrupts
            pass

    GPIO.output(self.enable_pin, GPIO.HIGH)


if __name__ == "__main__":
    pass

