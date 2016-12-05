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


def gcd(a, b):
    """Return greatest common divisor using Euclid's Algorithm."""
    while b:      
        a, b = b, a % b
    return a


def lcm(a, b):
    """Return lowest common multiple."""
    return a * b // gcd(a, b)


def pos_move(self):
    """Move axis"""

    # Only available for MotorControlXY object
    if self.node_name == 'MotorControlXY':
        try:
            _pos_move_two(self)
        except BaseException as e:
            print(e)
            _pos_move_one(self)  # Retry if DMA controls blocks are too high
    else:
        _pos_move_one(self)


def _pos_move_one(self):
    """Move one axis at a time."""

    for A in self.mode:
        # Direction : 1 for counter clockwise / 0 for clockwise
        if self.delta[A] != 0:
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


def _pos_move_two(self):
    """ Move axis x and y at the same time """
   
    # If only one axis has 0 mouvement
    if not self.delta[X] or not self.delta[Y]:
        pos_move_one(self)
    else:
        for A in self.mode:
            # Direction : 1 for counter clockwise / 0 for clockwise
            if self.delta[A] != 0:
                self.direction[A] = 1 if self.delta[A] < 0 else 0
                self.gpio.write(self.dir_pin[A], self.direction[A])

        self.nb_pulse[X] = abs(self.delta[X])
        self.nb_pulse[Y] = abs(self.delta[Y])

        # Create time pulses for x and y
        dt_x = soft_move(self.nb_pulse[X], self.f_max[X], self.f_min[X], self.max_slope[X])
        dt_y = soft_move(self.nb_pulse[Y], self.f_max[Y], self.f_min[Y], self.max_slope[Y])
        
        gen2clk(self, X, Y, dt_x, dt_y)






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
        print('soft_move received an invalid argument')
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
        print('Error - frequency trapeze has incorrect length ({0}), \
                            should be {1}'.format(len(dF), nb_pulse))
        return None

    return [int(500000/f) for f in dF]




def getIDX(array):
    """Get the start index and the end index of the smallest consecutive pulse if any"""
    
    index_low = []
    index_high = []

    len_array = len(array)
    ctr = 0
    detected = False



    min_nb = min(array)

    if array.count(min_nb) < 6:
        return [index_low, index_high]

    ctr = 0
    for i in range(len(array)):
        if ctr == 0 and array[i] == min_nb:
            index_low_dummy = i
            ctr = ctr + 1

        if ctr and array[i] == min_nb:
            index_high_dummy = i

    index_low.append(index_low_dummy)
    index_high.append(index_high_dummy)

    return [index_low, index_high]



def gen2clk(self, id_x, id_y, dt_x, dt_y):

    # Double the array to account for pulse on and off
    real_dt_x = []
    real_dt_y = []
    for i in dt_x:
        real_dt_x.append(i)
        real_dt_x.append(i)

    for i in dt_y:
        real_dt_y.append(i)
        real_dt_y.append(i)

    dt_x = real_dt_x
    dt_y = real_dt_y


    # Initialize variables
    generic_wave = [[],[]]
    FORMAT_ID = 0
    PULSE_ID  = 1
    pulse_arr_ctr = 0

    MAX_ARRAY_PULSE = 100

    NORMAL = 0
    REPEAT = 1

    pulse_x_len = len(dt_x)
    pulse_y_len = len(dt_y)
    IDX_LOW  = 0
    IDX_HIGH = 1

    idx_plateau_x = getIDX(dt_x)
    idx_plateau_y = getIDX(dt_y)

    plateau_x = True if idx_plateau_x[0] else False
    plateau_y = True if idx_plateau_y[0] else False


    # Mode 0
    if not plateau_x and not plateau_y:
        #print('MODE 0 -------------------------------------------------')
        ctr_x = 0;
        ctr_y = 0;

        usec_x = dt_x[ctr_x]
        usec_y = dt_y[ctr_y]

        generic_wave[PULSE_ID].append([])
        generic_wave[FORMAT_ID].append(NORMAL)

        if usec_x < usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            usec_y = usec_y - usec_x
            ctr_x = ctr_x + 1
            usec_x = dt_x[ctr_x]

        elif usec_x == usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            ctr_x = ctr_x + 1
            ctr_y = ctr_y + 1
            usec_x = dt_x[ctr_x]
            usec_y = dt_y[ctr_y]

        else:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_y))
            usec_x = usec_x - usec_y
            ctr_y = ctr_y + 1
            usec_y = dt_y[ctr_y]


        if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
            pulse_arr_ctr = pulse_arr_ctr + 1
            generic_wave[PULSE_ID].append([])
            generic_wave[FORMAT_ID].append(NORMAL)

        # Loop through the smallest array of pulse
        while ctr_x < pulse_x_len and ctr_y < pulse_y_len:

            isOdd_x = ctr_x%2
            isOdd_y = ctr_y%2
                    
            usec = usec_x if usec_x <= usec_y else usec_y

            if isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse((1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), 0, usec))
            elif isOdd_x and not isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 1<<self.clock_pin[id_y], usec))
            elif not isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 1<<self.clock_pin[id_x], usec))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec))

            # Update ctr_x, ctr_y, usec_x and usec_y accordingly
            if usec_x < usec_y:
                usec_y = usec_y - usec_x
                ctr_x = ctr_x + 1

                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]

            elif usec_x == usec_y:
                ctr_x = ctr_x + 1
                ctr_y = ctr_y + 1
                        
                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]

            else:
                usec_x = usec_x - usec_y
                ctr_y = ctr_y + 1
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]




            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

        # Append the rest of the pulses of x if any
        if ctr_x < pulse_x_len:
            print(dt_x[ctr_x:])
            for i in dt_x[ctr_x:]:
                isOdd_x = ctr_x%2

                if isOdd_x:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, i))
                else:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], i))

                ctr_x = ctr_x + 1
                if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[PULSE_ID].append([])
                    generic_wave[FORMAT_ID].append(NORMAL)


        # Append the rest of the pulses of y if any
        if ctr_y < pulse_y_len:
            for i in dt_y[ctr_y:]:
                isOdd_y = ctr_y%2

                if isOdd_y:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, i))
                else:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], i))

                ctr_y = ctr_y + 1
                if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[PULSE_ID].append([])
                    generic_wave[FORMAT_ID].append(NORMAL)


    elif plateau_x and not plateau_y:
        print('MODE 1 -------------------------------------------------')
        ctr_x = 0;
        ctr_y = 0;

        usec_x = dt_x[ctr_x]
        usec_y = dt_y[ctr_y]

        generic_wave[PULSE_ID].append([])
        generic_wave[FORMAT_ID].append(NORMAL)

        if usec_x < usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            usec_y = usec_y - usec_x
            ctr_x = ctr_x + 1
            usec_x = dt_x[ctr_x]

        elif usec_x == usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            ctr_x = ctr_x + 1
            ctr_y = ctr_y + 1
            usec_x = dt_x[ctr_x]
            usec_y = dt_y[ctr_y]

        else:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_y))
            usec_x = usec_x - usec_y
            ctr_y = ctr_y + 1
            usec_y = dt_y[ctr_y]


        if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
            pulse_arr_ctr = pulse_arr_ctr + 1
            generic_wave[PULSE_ID].append([])
            generic_wave[FORMAT_ID].append(NORMAL)

        # Loop through the smallest array of pulse
        while ctr_x < pulse_x_len and ctr_y < pulse_y_len:
            print('0')
            print(ctr_y)
            print(pulse_y_len)
            isOdd_x = ctr_x%2
            isOdd_y = ctr_y%2

            usec = usec_x if usec_x <= usec_y else usec_y

            if isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse((1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), 0, usec))
            elif isOdd_x and not isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 1<<self.clock_pin[id_y], usec))
            elif not isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 1<<self.clock_pin[id_x], usec))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec))

            # Update ctr_x, ctr_y, usec_x and usec_y accordingly
            if usec_x < usec_y:
                usec_y = usec_y - usec_x
                ctr_x = ctr_x + 1

                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]

            elif usec_x == usec_y:
                ctr_x = ctr_x + 1
                ctr_y = ctr_y + 1
                        
                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]

            else:
                usec_x = usec_x - usec_y
                ctr_y = ctr_y + 1
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]


            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

        # If there is some pulse left to send
        if ctr_x < pulse_x_len and usec_x != dt_x[ctr_x]:
            isOdd_x = ctr_x%2


            if isOdd_x:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, usec_x))
                #ramp_up.append(pigpio.pulse(1<<self.clock_pin[id_x], 0, usec_x))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], usec_x))
                #ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[id_x], usec_x))

            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

            ctr_x = ctr_x + 1


        # Append the rest of the pulses of x if any
        if ctr_x < pulse_x_len:

            # TODO change so that multiple plateau works
            while ctr_x < pulse_x_len:
                print('1')
                #print('Yoshi')
                #print(ctr_x)
                #print(idx_plateau_x[IDX_LOW][0])
                #print(idx_plateau_x[IDX_HIGH][0])

                isOdd_x = ctr_x%2

                if ctr_x >= idx_plateau_x[IDX_LOW][0] and ctr_x < idx_plateau_x[IDX_HIGH][0] and not isOdd_x:
                    #print('********************************************')
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append((idx_plateau_x[IDX_HIGH][0]-ctr_x+1)/2)


                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], dt_x[ctr_x]))
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, dt_x[ctr_x]))

                    ctr_x = idx_plateau_x[IDX_HIGH][0]
                    #print('Ricardo')
                    #print(ctr_x)
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append(NORMAL)



                else:

                    isOdd_x = ctr_x%2

                    if isOdd_x:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, dt_x[ctr_x]))
                        #ramp_up.append(pigpio.pulse(1<<self.clock_pin[id_x], 0, i))
                    else:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], dt_x[ctr_x]))
                        #ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[id_x], i))

                    if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                        pulse_arr_ctr = pulse_arr_ctr + 1
                        generic_wave[PULSE_ID].append([])
                        generic_wave[FORMAT_ID].append(NORMAL)

                ctr_x = ctr_x + 1






        # If there is some pulse left to send
        if ctr_y < pulse_y_len and usec_y != dt_y[ctr_y]:
            print('Residual pulse Y !!!!!!!!!!!!!!!!!!!!!!')
            isOdd_y = ctr_y%2

            if isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, usec_y))
                #ramp_up.append(pigpio.pulse(1<<self.clock_pin[id_x], 0, usec_x))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], usec_y))
                #ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[id_x], usec_x))

            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

            ctr_y = ctr_y + 1

        #Append the rest of the pulses of x if any
        if ctr_y < pulse_y_len:

            # TODO change so that multiple plateau works
            while ctr_y < pulse_y_len:
                print('1')
                #print('Yoshi')
                #print(ctr_x)
                #print(idx_plateau_y[IDX_LOW][0])
                #print(idx_plateau_y[IDX_HIGH][0])



                isOdd_y = ctr_y%2

                if isOdd_y:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, dt_y[ctr_y]))
                    #ramp_up.append(pigpio.pulse(1<<self.clock_pin[id_x], 0, i))
                else:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], dt_y[ctr_y]))
                    #ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[id_x], i))

                if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[PULSE_ID].append([])
                    generic_wave[FORMAT_ID].append(NORMAL)

                ctr_y = ctr_y + 1



    # Mode 2
    elif not plateau_x and plateau_y:
        print('MODE 2 -------------------------------------------------')

        ctr_x = 0;
        ctr_y = 0;

        usec_x = dt_x[ctr_x]
        usec_y = dt_y[ctr_y]

        generic_wave[PULSE_ID].append([])
        generic_wave[FORMAT_ID].append(NORMAL)

        if usec_x < usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            usec_y = usec_y - usec_x
            ctr_x = ctr_x + 1
            usec_x = dt_x[ctr_x]

        elif usec_x == usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            ctr_x = ctr_x + 1
            ctr_y = ctr_y + 1
            usec_x = dt_x[ctr_x]
            usec_y = dt_y[ctr_y]

        else:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_y))
            usec_x = usec_x - usec_y
            ctr_y = ctr_y + 1
            usec_y = dt_y[ctr_y]


        if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
            pulse_arr_ctr = pulse_arr_ctr + 1
            generic_wave[PULSE_ID].append([])
            generic_wave[FORMAT_ID].append(NORMAL)

        # Loop through the smallest array of pulse
        while ctr_x < pulse_x_len and ctr_y < pulse_y_len:
            print('0')
            isOdd_x = ctr_x%2
            isOdd_y = ctr_y%2

            usec = usec_x if usec_x <= usec_y else usec_y

            if isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse((1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), 0, usec))
            elif isOdd_x and not isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 1<<self.clock_pin[id_y], usec))
            elif not isOdd_x and isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 1<<self.clock_pin[id_x], usec))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec))

            # Update ctr_x, ctr_y, usec_x and usec_y accordingly
            if usec_x < usec_y:
                usec_y = usec_y - usec_x
                ctr_x = ctr_x + 1

                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]

            elif usec_x == usec_y:
                ctr_x = ctr_x + 1
                ctr_y = ctr_y + 1
                if ctr_x < pulse_x_len:
                    usec_x = dt_x[ctr_x]
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]

            else:
                usec_x = usec_x - usec_y
                ctr_y = ctr_y + 1
                if ctr_y < pulse_y_len:
                    usec_y = dt_y[ctr_y]


            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)
        
        # If there is some pulse left to send
        if ctr_y < pulse_y_len and usec_y != dt_y[ctr_y]:
            isOdd_y = ctr_y%2

            if isOdd_y:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, usec_y))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], usec_y))

            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

            ctr_y = ctr_y + 1

        #Append the rest of the pulses of x if any
        if ctr_y < pulse_y_len:

            # TODO change so that multiple plateau works
            while ctr_y < pulse_y_len:

                isOdd_y = ctr_y%2

                if ctr_y >= idx_plateau_y[IDX_LOW][0] and ctr_y < idx_plateau_y[IDX_HIGH][0] and not isOdd_y:
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append((idx_plateau_y[IDX_HIGH][0]-ctr_y+1)/2)


                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], dt_y[ctr_y]))
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, dt_y[ctr_y]))

                    ctr_y = idx_plateau_y[IDX_HIGH][0]
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append(NORMAL)



                else:

                    isOdd_y = ctr_y%2

                    if isOdd_y:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, dt_y[ctr_y]))
                    else:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], dt_y[ctr_y]))

                    if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                        pulse_arr_ctr = pulse_arr_ctr + 1
                        generic_wave[PULSE_ID].append([])
                        generic_wave[FORMAT_ID].append(NORMAL)

                ctr_y = ctr_y + 1



        # If there is some pulse left to send
        if ctr_x < pulse_x_len and usec_x != dt_x[ctr_x]:
            isOdd_x = ctr_x%2


            if isOdd_x:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, usec_x))
            else:
                generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], usec_x))

            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)

            ctr_x = ctr_x + 1


        # Append the rest of the pulses of x if any
        if ctr_x < pulse_x_len:

            # TODO change so that multiple plateau works
            while ctr_x < pulse_x_len:

                isOdd_x = ctr_x%2

                if isOdd_x:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, dt_x[ctr_x]))
                else:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], dt_x[ctr_x]))

                if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[PULSE_ID].append([])
                    generic_wave[FORMAT_ID].append(NORMAL)

                ctr_x = ctr_x + 1




    # Mode 3
    elif plateau_x and plateau_y:
        print('MODE 3 -------------------------------------------------')
        ctr_x = 0;
        ctr_y = 0;

        usec_x = dt_x[ctr_x]
        usec_y = dt_y[ctr_y]

        generic_wave[PULSE_ID].append([])
        generic_wave[FORMAT_ID].append(NORMAL)

        if usec_x < usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            usec_y = usec_y - usec_x
            ctr_x = ctr_x + 1
            usec_x = dt_x[ctr_x]

        elif usec_x == usec_y:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_x))
            ctr_x = ctr_x + 1
            ctr_y = ctr_y + 1
            usec_x = dt_x[ctr_x]
            usec_y = dt_y[ctr_y]

        else:
            generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec_y))
            usec_x = usec_x - usec_y
            ctr_y = ctr_y + 1
            usec_y = dt_y[ctr_y]


        if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
            pulse_arr_ctr = pulse_arr_ctr + 1
            generic_wave[PULSE_ID].append([])
            generic_wave[FORMAT_ID].append(NORMAL)




        # Loop through the smallest array of pulse
        while ctr_x < pulse_x_len and ctr_y < pulse_y_len:

            isOdd_x = ctr_x%2
            isOdd_y = ctr_y%2



            # Must be the longuest period
            ctr_long = ctr_x if dt_x[ctr_x] >= dt_y[ctr_y] else ctr_y
            ctr_small = ctr_y if dt_y[ctr_y] < dt_x[ctr_x] else ctr_x
            dt_long  = dt_x if dt_x[ctr_x] >= dt_y[ctr_y] else dt_y
            dt_small = dt_y if dt_y[ctr_y] < dt_x[ctr_x] else dt_x
            isOdd_long = ctr_long%2
            isOdd_small = ctr_small%2

            lcm_nb = lcm(dt_x[ctr_x], dt_y[ctr_y])
            min_pulse_long  = (lcm_nb / dt_long[ctr_long]) * 2
            min_pulse_small = (lcm_nb / dt_small[ctr_small]) * 2

            idx_plateau_long  = idx_plateau_x if ctr_long == ctr_x else idx_plateau_y 
            idx_plateau_small = idx_plateau_y if ctr_small == ctr_y else idx_plateau_x

            if ctr_long >= idx_plateau_long[IDX_LOW][0] and ctr_long < idx_plateau_long[IDX_HIGH][0] and \
                ctr_small >= idx_plateau_small[IDX_LOW][0] and ctr_small < idx_plateau_small[IDX_HIGH][0] and \
                not isOdd_long and (idx_plateau_long[IDX_HIGH][0] - ctr_long) >= min_pulse_long and (idx_plateau_small[IDX_HIGH][0] - ctr_small) >= min_pulse_small:

                # Add new
                generic_wave[PULSE_ID].append([])
                pulse_arr_ctr = pulse_arr_ctr + 1
                max_ctr_long = ctr_long + min_pulse_long
                ctr_long_ini = ctr_long
                ctr_small_ini = ctr_small




                while (ctr_long < max_ctr_long):
                    isOdd_x = ctr_x%2
                    isOdd_y = ctr_y%2
                    
                    usec = usec_x if usec_x <= usec_y else usec_y 

                    if isOdd_x and isOdd_y:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse((1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), 0, usec))
                    elif isOdd_x and not isOdd_y:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 1<<self.clock_pin[id_y], usec))
                    elif not isOdd_x and isOdd_y:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 1<<self.clock_pin[id_x], usec))
                    else:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec))

                    # Update ctr_x, ctr_y, usec_x and usec_y accordingly
                    if usec_x < usec_y:
                        usec_y = usec_y - usec_x
                        ctr_x = ctr_x + 1

                        if ctr_x < pulse_x_len:
                            usec_x = dt_x[ctr_x]

                    elif usec_x == usec_y:
                        ctr_x = ctr_x + 1
                        ctr_y = ctr_y + 1
                        
                        if ctr_x < pulse_x_len:
                            usec_x = dt_x[ctr_x]
                        if ctr_y < pulse_y_len:
                            usec_y = dt_y[ctr_y]

                    else:
                        usec_x = usec_x - usec_y
                        ctr_y = ctr_y + 1
                        if ctr_y < pulse_y_len:
                            usec_y = dt_y[ctr_y]

                    # Increment ctr_long and ctr_small
                    ctr_long = ctr_x if dt_x[ctr_x] >= dt_y[ctr_y] else ctr_y
                    ctr_small = ctr_y if dt_y[ctr_y] < dt_x[ctr_x] else ctr_x
                
                
                # Calculate number of repeated pulses
                


                len_repeat_long  = int((idx_plateau_long[IDX_HIGH][0] - ctr_long_ini) / min_pulse_long)
                len_repeat_small = int((idx_plateau_small[IDX_HIGH][0] - ctr_small_ini) / min_pulse_small)

                len_repeat = len_repeat_long if len_repeat_long <= len_repeat_small else len_repeat_small
                generic_wave[FORMAT_ID].append(len_repeat)

                generic_wave[PULSE_ID].append([])
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[FORMAT_ID].append(NORMAL)

                # Recalculate ctr_x and ctr_y
                ctr_x = ctr_x + ((len_repeat-1)*min_pulse_long) if ctr_x == ctr_long else ctr_x + ((len_repeat-1)*min_pulse_small)
                ctr_y = ctr_y + ((len_repeat-1)*min_pulse_long) if ctr_y == ctr_long else ctr_y + ((len_repeat-1)*min_pulse_small)


            else:

                usec = usec_x if usec_x <= usec_y else usec_y 

                if isOdd_x and isOdd_y:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse((1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), 0, usec))
                elif isOdd_x and not isOdd_y:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 1<<self.clock_pin[id_y], usec))
                elif not isOdd_x and isOdd_y:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 1<<self.clock_pin[id_x], usec))
                else:
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, (1<<self.clock_pin[id_x]) + (1<<self.clock_pin[id_y]), usec))

                # Update ctr_x, ctr_y, usec_x and usec_y accordingly
                if usec_x < usec_y:
                    usec_y = usec_y - usec_x
                    ctr_x = ctr_x + 1

                    if ctr_x < pulse_x_len:
                        usec_x = dt_x[ctr_x]

                elif usec_x == usec_y:
                    ctr_x = ctr_x + 1
                    ctr_y = ctr_y + 1
                    
                    if ctr_x < pulse_x_len:
                        usec_x = dt_x[ctr_x]
                    if ctr_y < pulse_y_len:
                        usec_y = dt_y[ctr_y]

                else:
                    usec_x = usec_x - usec_y
                    ctr_y = ctr_y + 1
                    if ctr_y < pulse_y_len:
                        usec_y = dt_y[ctr_y]


            if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                pulse_arr_ctr = pulse_arr_ctr + 1
                generic_wave[PULSE_ID].append([])
                generic_wave[FORMAT_ID].append(NORMAL)




        # Append the rest of the pulses of x if any
        if ctr_x < pulse_x_len:
            # TODO change so that multiple plateau works
            while ctr_x < pulse_x_len:

                isOdd_x = ctr_x%2

                if ctr_x >= idx_plateau_x[IDX_LOW][0] and ctr_x < idx_plateau_x[IDX_HIGH][0] and not isOdd_x:
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append((idx_plateau_x[IDX_HIGH][0]-ctr_x+1)/2)


                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], dt_x[ctr_x]))
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, dt_x[ctr_x]))

                    ctr_x = idx_plateau_x[IDX_HIGH][0]
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append(NORMAL)



                else:

                    isOdd_x = ctr_x%2

                    if isOdd_x:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_x], 0, dt_x[ctr_x]))
                    else:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_x], dt_x[ctr_x]))

                    if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                        pulse_arr_ctr = pulse_arr_ctr + 1
                        generic_wave[PULSE_ID].append([])
                        generic_wave[FORMAT_ID].append(NORMAL)

                ctr_x = ctr_x + 1



        #Append the rest of the pulses of x if any
        if ctr_y < pulse_y_len:
            # TODO change so that multiple plateau works
            while ctr_y < pulse_y_len:

                isOdd_y = ctr_y%2

                if ctr_y >= idx_plateau_y[IDX_LOW][0] and ctr_y < idx_plateau_y[IDX_HIGH][0] and not isOdd_y:
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append((idx_plateau_y[IDX_HIGH][0]-ctr_y+1)/2)


                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], dt_y[ctr_y]))
                    generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, dt_y[ctr_y]))

                    ctr_y = idx_plateau_y[IDX_HIGH][0]
                    generic_wave[PULSE_ID].append([])
                    pulse_arr_ctr = pulse_arr_ctr + 1
                    generic_wave[FORMAT_ID].append(NORMAL)



                else:

                    isOdd_y = ctr_y%2

                    if isOdd_y:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(1<<self.clock_pin[id_y], 0, dt_y[ctr_y]))
                    else:
                        generic_wave[PULSE_ID][pulse_arr_ctr].append(pigpio.pulse(0, 1<<self.clock_pin[id_y], dt_y[ctr_y]))

                    if len(generic_wave[PULSE_ID][pulse_arr_ctr]) == MAX_ARRAY_PULSE:
                        pulse_arr_ctr = pulse_arr_ctr + 1
                        generic_wave[PULSE_ID].append([])
                        generic_wave[FORMAT_ID].append(NORMAL)

                ctr_y = ctr_y + 1


    # Hack 
    if not generic_wave[PULSE_ID][-1]:
        generic_wave[PULSE_ID].pop()
        generic_wave[FORMAT_ID].pop()





    # Test for CBS before creating wave
    estimate_cbs = 0
    
    for pulse_array in generic_wave[PULSE_ID]:
        estimate_cbs = estimate_cbs + (2 * len(pulse_array))

    print('Estimated CBS = {0}'.format(estimate_cbs))

    if estimate_cbs >= 20000:
    #if estimate_cbs >= self.gpio.wave_get_max_cbs():
        print('Error estimation will max out CBS')

    self.gpio.exceptions = False
    # Create wave
    self.gpio.wave_clear()

    print('CBS = {0}'.format(self.gpio.wave_get_cbs()))
    print('MAX CBS = {0}'.format(self.gpio.wave_get_max_cbs()))


    wave_id_list = []
    wave_id_remove = []

    real_cbs = 0
    for ctr in range(len(generic_wave[PULSE_ID])):

        self.gpio.wave_add_new()
        self.gpio.wave_add_generic(generic_wave[PULSE_ID][ctr])
        print('CBS = {0}'.format(self.gpio.wave_get_cbs()))

        wave_id = self.gpio.wave_create()
        wave_id_remove.append(wave_id)
        # Pulses with a loop
        if generic_wave[FORMAT_ID][ctr]:
            print('repeat')
            print(generic_wave[FORMAT_ID][ctr])
            print(len(generic_wave[PULSE_ID][ctr]))
            m = generic_wave[FORMAT_ID][ctr] % 256
            n = generic_wave[FORMAT_ID][ctr] / 256
            
            wave_id_list.append(255)
            wave_id_list.append(0)
            wave_id_list.append(wave_id)
            wave_id_list.append(255)
            wave_id_list.append(1)
            wave_id_list.append(m)
            wave_id_list.append(n)

        # Normal pulses
        else:
            wave_id_list.append(wave_id)
        real_cbs = real_cbs + self.gpio.wave_get_cbs() 
    
    
    print('Calculated CBS = {0}'.format(real_cbs))

    print(wave_id_list)
    # Enable motor driver
    for A in range(len(self.sync)):
        for B in range(self.sync[A]): 
            self.gpio.write(self.enable_pin[A][B], pigpio.HIGH)


    # Send pulses to the motor driver
    self.gpio.wave_chain(wave_id_list)

    print('STARTING {0}'.format(self.node_name))

    while self.gpio.wave_tx_busy():
        self.rate.sleep()

    # Stop motor driver
    for A in range(len(self.sync)):
        for B in range(self.sync[A]): 
            self.gpio.write(self.enable_pin[A][B], pigpio.LOW)



    for A in self.mode:
        # Direction : 1 for counter clockwise / 0 for clockwise
        self.gpio.write(self.dir_pin[A], pigpio.HIGH)

    for wave_id in wave_id_remove:
        self.gpio.wave_delete(wave_id)




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
            ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[t]))
            ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[t]))
            ramp_down.append(pigpio.pulse(0, 1<<self.clock_pin[A],\
                                          dt[t+magic+plat_len]))
            ramp_down.append(pigpio.pulse(1<<self.clock_pin[A], 0,\
                                          dt[t+magic+plat_len]))

        plateau.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[magic]))
        plateau.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[magic]))


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

            ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], dt[0]))
            ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, dt[0]))


            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_up)
            wave_id1 = self.gpio.wave_create()

            wave_id_list = [wave_id1]

        else:
            for t in dt[:len(dt)/2]:
                ramp_up.append(pigpio.pulse(0, 1<<self.clock_pin[A], t))
                ramp_up.append(pigpio.pulse(1<<self.clock_pin[A], 0, t))
            for t in dt[len(dt)/2:]:
                ramp_down.append(pigpio.pulse(0, 1<<self.clock_pin[A], t))
                ramp_down.append(pigpio.pulse(1<<self.clock_pin[A], 0, t))

            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_up)
            wave_id1 = self.gpio.wave_create()
            self.gpio.wave_add_new()
            self.gpio.wave_add_generic(ramp_down)
            wave_id2 = self.gpio.wave_create()
            wave_id_list = [wave_id1, wave_id2]

    print('{0} wid : {1}'.format(self.node_name, wave_id_list))

    for B in range(self.sync[A]):
        self.gpio.write(self.enable_pin[A][B], pigpio.HIGH)

    self.gpio.wave_chain(wave_id_list)

    print('STARTING {0}'.format(self.node_name))

    while self.gpio.wave_tx_busy():
        self.rate.sleep()

    for B in range(self.sync[A]):
        self.gpio.write(self.enable_pin[A][B], pigpio.LOW)


    for A in self.mode:
        # Direction : 1 for counter clockwise / 0 for clockwise
        print('Allah Wackba')
        print(A)
        print(self.dir_pin[A])
        self.gpio.write(self.dir_pin[A], pigpio.HIGH)



    # Delete existing wave IDs
    if wave_id1 is not None:
        self.gpio.wave_delete(wave_id1)

    if wave_id2 is not None:
        self.gpio.wave_delete(wave_id2)

    if wave_id3 is not None:
        self.gpio.wave_delete(wave_id3)

    print('{0} DONE'.format(self.node_name))


# Main function
if __name__ == '__main__':
    mcxy = MotorControlXY()


    dt_x = soft_move(3, 100, 50, 4)
    dt_y = soft_move(10, 10, 2, 4)

    # Test Mode 1
    dt_x = [50, 25, 12, 12, 12, 12, 12, 12, 12, 25, 50]
    dt_y = [50, 26, 11]



    # Test Mode 3
    #dt_x = [50, 25, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 25, 50]
    #dt_y = [50, 25, 20, 18,  7,  7,  7,  7,  7,  7,  7, 30, 50]

    #dt_x = [10, 4, 3, 3, 3, 3, 3, 4, 10]
    #dt_y = [10, 6, 2, 2, 2, 2, 4,10]
    #dt_x = [10, 4, 3, 3, 3, 3, 3, 4, 10, 20,30,40,50]
    #dt_y = [10, 6, 2, 2, 2, 2, 4,10]


    dt_x = [10, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
    dt_y = [10, 6, 2, 2, 2, 2, 4,5]


    print(dt_x)
    print(dt_y)

    #gen2clk(mcxy,0, 1, dt_x, dt_y)

