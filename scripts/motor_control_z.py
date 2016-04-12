#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
import pigpio
from platform_control.msg import IntList
import rospy
from std_msgs.msg import String
import trajectory
from trajectory import Z0, Z1, Z2

class MotorControlZ(BaseMotorControl):
    """Multiple attributes are lists of three elements --> [Z0, Z1, Z2]"""

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_Z is [ID, int_z], where ID is 0, 1 or 2
        self.subscriber = rospy.Subscriber('Pulse_Z', IntList, self.callback_pos)

        # Frequency trapeze constants
        self.f_max     = [5000, 6000, 6000]  # Tested and works at 15 KHz
        self.f_min     = [500 ,  500, 500 ]
        self.max_slope = [10  ,    5, 5   ]
        self.f_init    = [4000, 4000, 4000]  # Valid freq (Hz): 8000 4000 2000 1600
        self.init_dir  = pigpio.LOW
        self.init_list = []

        # Position control
        self.mode      = [Z0, Z1, Z2]  # Used for Z0, Z1 and Z2
        self.modes     = [Z0, Z1, Z2]
        self.sync      = [1,  1,  1 ]
        self.delta     = [0,  0,  0 ]
        self.direction = [0,  0,  0 ]
        self.nb_pulse  = [0,  0,  0 ]

        # GPIO pins
        self.enable_pin = [[26], [7 ], [22]]  # pins [37], [26] and [15]
        self.limit_sw   = [[16], [20], [21]]  # pins [36], [38] and [40]
        self.clock_pin  = [ 13 ,  25 ,  17 ]  # pins  33 ,  22  and  11
        self.dir_pin    = [ 19 ,  8  ,  27 ]  # pins  35 ,  24  and  13

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            assert len(data.data) == 2
            mode, delta = data.data
            delta *= -1
            assert mode in self.modes
            assert type(delta) == int
            assert abs(delta) < 65536
            self.mode = [mode]
            self.delta[mode] = delta

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for Z: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        if self.delta[mode]:
            trajectory.pos_move(self)
            self.done_module.publish(self.node_name)

    def callback_limit_sw_z0(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
        self.cb_sw_z0.cancel()
        self.init_list.remove("00")

    def callback_limit_sw_z1(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[1][0], pigpio.LOW)
        self.cb_sw_z1.cancel()
        self.init_list.remove("10")

    def callback_limit_sw_z2(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[2][0], pigpio.LOW)
        self.cb_sw_z2.cancel()
        self.init_list.remove("20")

    def set_cb_sw(self):
        self.cb_sw_z0 = self.gpio.callback(self.limit_sw[0][0], pigpio.FALLING_EDGE, \
                                                 self.callback_limit_sw_z0)
        self.cb_sw_z1 = self.gpio.callback(self.limit_sw[1][0], pigpio.FALLING_EDGE, \
                                                 self.callback_limit_sw_z1)
        self.cb_sw_z2 = self.gpio.callback(self.limit_sw[2][0], pigpio.FALLING_EDGE,  \
                                                  self.callback_limit_sw_z2)


# Main function
if __name__ == '__main__':
    try:
        mcz = MotorControlZ()
        mcz.listener()

    except rospy.ROSInterruptException as e:
        print(e)

