#!/usr/bin/python

# Imports
import pigpio

# Constants
X = 0
Y = 1
Z0 = 0
Z1 = 1
Z2 = 2
SP = 0
MP = 0

# Functions
def pos_move(self):
    """
    Generates a clock signal on the GPIO pins of the motor_control node object.
    """

    for A in self.mode:
        # Direction : 1 for counter clockwise / 0 for clockwise
        self.direction[A] = 1 if self.delta[A] < 0 else 0
        self.gpio.write(self.dir_pin[A], self.direction[A])

        # Movement
        self.nb_pulse[A] = abs(self.delta[A])

        # Generate clock
        if self.nb_pulse[A]:
            dt = soft_move(self.nb_pulse[A], self.f_max[A], \
                           self.f_min[A], self.max_slope[A])
            if dt:
                gen_single_clock(self, A, dt)


def soft_move(nb_pulse, f_max, f_min, max_slope):
    """
    Creates a list of time values to use as PWM to move the platform.
    Input:  Number of pulses and frequency constraints
    Output: The time array (in microseconds) associated to frequencies
            that form a trapeze (ramp up, plateau, ramp down).
    """

    # Return nothing if any argument is invalid
    if not nb_pulse:
        return None

    try:
        assert nb_pulse > 0
        assert f_min > 0
        assert f_max >= f_min
        assert max_slope > 0
    except AssertionError:
        print("soft_move received an invalid argument")
        return []

    magic = int((f_max-f_min)/max_slope)
    plat_len = nb_pulse-2*magic

    if plat_len > 0:
        dF = [i*max_slope+f_min for i in range(magic)]
        dF = dF + [f_max]*plat_len + dF[::-1]

    elif nb_pulse == 1:
        dF = [f_min]

    else:
        dF = [i*max_slope+f_min for i in range(nb_pulse/2)]
        buf = [dF[-1]] if nb_pulse&1 else []
        dF = dF + buf + dF[::-1]

    try:
        assert len(dF) == nb_pulse
    except AssertionError:
        print("Error - frequency trapeze has incorrect length ({0}), \
                            should be {1}".format(len(dF), nb_pulse))
        return None

    return [int(500000/f) for f in dF]


def gen_single_clock(self, A, dt):
    magic = int((self.f_max[A]-self.f_min[A])/self.max_slope[A])
    plat_len = self.nb_pulse[A]-2*magic

    self.gpio.wave_clear()

    wave_id1 = None
    wave_id2 = None
    wave_id3 = None
    ramp_up = []
    plateau = []
    ramp_down = []

    if plat_len > 0:
        for t in range(magic):
            ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[t]))
            ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[t]))
            ramp_down.append(pigpio.pulse(1<<self.clock_pin[A], 0,\
                                          dt[t+magic+plat_len]))
            ramp_down.append(pigpio.pulse(0, 1<<self.clock_pin[A],\
                                          dt[t+magic+plat_len]))

        plateau.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[magic]))
        plateau.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[magic]))


        if magic != 0:
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_up)
            wave_id1 = self.gpio.wave_create()
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(plateau)
            wave_id2 = self.gpio.wave_create()
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_down)
            wave_id3 = self.gpio.wave_create()

            m = plat_len % 256
            n = plat_len / 256

            wave_id_list = [wave_id1,       # Transmit wave
                            255, 0,         # Start loop
                                wave_id2,   # Transmit wave
                            255, 1, m, n,   # Loop m + 256*n times
                            wave_id3]       # Transmit wave

        else:
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(plateau)
            wave_id1 = self.gpio.wave_create()

            m = plat_len % 256
            n = plat_len / 256

            wave_id_list = [255, 0,         # Start loop
                            wave_id1,       # Transmit wave
                            255, 1, m, n]   # Loop m + 256*n times

    else:
        if len(dt) == 1:

            ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[0]))
            ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[0]))


            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_up)
            wave_id1 = self.gpio.wave_create()

            wave_id_list = [wave_id1]

        else:
            for t in dt[:len(dt)/2]:
                ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, t))
                ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], t))
            for t in dt[len(dt)/2:]:
                ramp_down.append(pigpio.pulse(1<<self.clock_pin[A], 0, t))
                ramp_down.append(pigpio.pulse(0, 1<<self.clock_pin[A], t))

            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_up)
            wave_id1 = self.gpio.wave_create()
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_down)
            wave_id2 = self.gpio.wave_create()
            wave_id_list = [wave_id1, wave_id2]

    print("{0} wid : {1}".format(self.node_name, wave_id_list))

    for B in range(self.sync[A]):
        self.gpio.write(self.enable_pin[A][B], pigpio.HIGH)

    self.gpio.wave_chain(wave_id_list)

    print("STARTING {0}".format(self.node_name))

    while self.gpio.wave_tx_busy():
        self.rate.sleep()

    for B in range(self.sync[A]):
        self.gpio.write(self.enable_pin[A][B], pigpio.LOW)

    # Delete existing wave IDs
    if wave_id1 is not None:
        self.gpio.wave_delete(wave_id1)

    if wave_id2 is not None:
        self.gpio.wave_delete(wave_id2)

    if wave_id3 is not None:
        self.gpio.wave_delete(wave_id3)

    print("{0} DONE".format(self.node_name))


# TODO in Fall 2016
# def gen_double_clock(self, dt_x, dt_y):
#     print("gen_double_clock not yet implemented!")
#     print("Moving X axis, then Y axis...")

#     self.single_axis_mode = X
#     gen_single_clock(self, dt_x)
#     self.single_axis_mode = Y
#     gen_single_clock(self, dt_y)


#     if not (N_x and N_y):
#         print("Movement only in one axis, use single method")
#         continue

#     dt_x = soft_move(N_x, f_max_x, f_min_x, max_slope_x)
#     dt_y = soft_move(N_y, f_max_y, f_min_y, max_slope_y)

#     dt2x = [x for pair in zip(dt_x, dt_x) for x in pair]
#     dt2y = [y for pair in zip(dt_y, dt_y) for y in pair]

#     dt2x_copy = list(dt2x)
#     dt2y_copy = list(dt2y)
#     dt2x_copy.append(0)
#     dt2y_copy.append(0)

#     ind_x, ind_y = 0, 0
#     last_x, last_y = dt2x[ind_x], dt2y[ind_y]
#     out = []

#     while ind_y <= len(dt2y)-1 and ind_x <= len(dt2x)-1:
#         on = ('X' if not ind_x&1 else '') + ('Y' if not ind_y&1 else '')
#         off = ('X' if ind_x&1 else '') + ('Y' if ind_y&1 else '')

#         if last_x == last_y:
#             delay = last_x
#             ind_x += 1
#             ind_y += 1
#             last_x = dt2x_copy[ind_x]
#             last_y = dt2y_copy[ind_y]
#         elif last_x > last_y:
#             delay = last_y
#             last_x -= last_y
#             ind_y += 1
#             last_y = dt2y_copy[ind_y]
#         else:
#             delay = last_x
#             last_y -= last_x
#             ind_x += 1
#             last_x = dt2x_copy[ind_x]
#
#         out.append((on, off, delay))

#     if dt2x[ind_x:]:
#         left_x = dt2x[ind_x:]
#         left_x[0] -= delay
#         for x in left_x:
#             on = ('X' if not ind_x&1 else '')
#             off = ('X' if ind_x&1 else '') + 'Y'
#             out.append((on, off, x))
#             ind_x += 1

