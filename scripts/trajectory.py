#!/usr/bin/env python

# Imports
import time
import RPi.GPIO as GPIO

# Functions
def soft_move(N, f_max, f_min, plat_size, max_slope):
    """
    Creates a list of time values to use as PWM to move the platform according to
    multiple input parameters.
    Inputs: - N             Number of motor steps  needed
            - f_max         Maximal frequency (Hz)
            - f_min         Minimal frequency (Hz)
            - plat_size     Size of plateau (0 to 1)
            - max_slope     Maximum value to prevent harsh transitions

    Output: - dt            The time array associated to frequencies that
                            form a trapeze (ramp up, plateau, ramp down).
    """

    # Return nothing if any argument is invalid
    for arg in locals().values():
        if arg <= 0:
            print("Error - soft_move invalid argument : {0} <= 0".format(arg))
            return None

    # Calculate center index
    center_ind = round(N/2)

    # Calculate beginning of plateau index
    plat_ind = round(center_ind-round(plat_size*N)/2)

    # if no ramp, return slowest movement
    if plat_ind < 1:
        print("plat_ind < 1")
        dt = [1/float(f_min)]*N
        return(dt)

    # Calculate frequency slope
    slope = min((f_max-f_min)/(plat_ind), max_slope)

    # Generate instantaneous frequency points
    dF = [slope*i+f_min for i in range(int(plat_ind+1))]
    dF = dF + [dF[-1]]*int(center_ind-plat_ind-1)
    buf = [dF[-1]] if N&1 else []   # Account for even/odd number of points
    dF = dF[:int(center_ind)]
    dF = dF + buf + dF[::-1]

    # Time of half period
    dt = [0.5/f for f in dF]

    return dt

def axis_move(enable_pin, control_pin, N, fmax, fmin, plat_size, max_slope):

    dt = soft_move(N, fmax, fmin, plat_size, max_slope)

    GPIO.output(enable_pin, GPIO.LOW)
    time.sleep(0.01)

    for x in range(N):
        time_begin = time.clock()
        GPIO.output(control_pin, GPIO.HIGH)

        while(time.clock() - time_begin < dt[x]):
            pass

        time_begin = time.clock()

        GPIO.output(control_pin, GPIO.LOW)

        while(time.clock() - time_begin < dt[x]):
            pass

    GPIO.output(enable_pin, GPIO.HIGH)


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
        axis_move(self.enable_pin, self.control_pin, self.nb_pulse,\
                self.f_max, self.f_min, self.plat_size, self.max_slope)


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
            pass

        time_begin = time.clock()
        GPIO.output(self.control_pin, GPIO.LOW)

        while(time.clock() - time_begin < dt):
            pass

    GPIO.output(self.enable_pin, GPIO.HIGH)


if __name__ == "__main__":
    N = 100
    f_max = 4000
    f_min = 500
    plat_size = 0.8
    max_slope = 5000

    dt = soft_move(N, f_max, f_min, plat_size, max_slope)
    for t in dt:
        print(t)

