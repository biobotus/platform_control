#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
import pigpio
from platform_control.msg import IntList
import rospy
from std_msgs.msg import String
import trajectory
from trajectory import X, Y

class MotorControlXY(BaseMotorControl):
    """Multiple attributes are lists of two elements --> [X, Y]"""

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_XY is [int_x, int_y]
        self.subscriber = rospy.Subscriber('Pulse_XY', IntList, self.callback_pos)

        # Frequency trapeze constants
        self.f_max     = [12000, 14000]
        self.f_min     = [  500,   500]
        self.max_slope = [    5,     5]
        self.f_init    = 2000
        self.init_dir  = pigpio.HIGH
        self.init_list = []

        # Position control
        self.mode       = [X, Y]
        self.modes      = [X, Y]
        self.sync       = [2, 1] # Axis X has 2 motors that have to be synced
        self.delta      = [0, 0]
        self.direction  = [0, 0]
        self.nb_pulse   = [0, 0]

        # GPIO pins
        self.enable_pin = [[14, 15], [11]]  # pins [8,  10] and [23]
        self.limit_sw   = [[23, 24], [12]]  # pins [16, 18] and [32]
        self.clock_pin  = [  5     ,  10 ]  # pins  29      and 19
        self.dir_pin    = [  6     ,   9 ]  # pins  31      and 21

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            self.delta = data.data
            assert len(self.delta) == 2
            assert type(self.delta[X]) == int
            assert type(self.delta[Y]) == int
            assert abs(self.delta[X]) < 65536
            assert abs(self.delta[Y]) < 65536

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for X, Y: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        trajectory.pos_move(self)
        self.done_module.publish(self.node_name)

    def callback_limit_sw_x0(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
        self.cb_sw_x0.cancel()
        try:
            self.init_list.remove("00")
            print("sw_x0 pressed")
        except ValueError:
            pass

    def callback_limit_sw_x1(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[0][1], pigpio.LOW)
        self.cb_sw_x1.cancel()
        try:
            self.init_list.remove("01")
            print("sw_x1 pressed")
        except ValueError:
            pass

    def callback_limit_sw_y(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[1][0], pigpio.LOW)
        self.cb_sw_y.cancel()
        try:
            self.init_list.remove("10")
            print("sw_y pressed")
        except ValueError:
            pass

    def set_cb_sw(self):
        self.cb_sw_x0 = self.gpio.callback(self.limit_sw[0][0], pigpio.FALLING_EDGE, \
                                                 self.callback_limit_sw_x0)
        self.cb_sw_x1 = self.gpio.callback(self.limit_sw[0][1], pigpio.FALLING_EDGE, \
                                                 self.callback_limit_sw_x1)
        self.cb_sw_y = self.gpio.callback(self.limit_sw[1][0], pigpio.FALLING_EDGE,  \
                                                  self.callback_limit_sw_y)



# Main function
if __name__ == '__main__':
    try:
        mcxy = MotorControlXY()
        mcxy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

