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
        self.f_max = [8000, 14000]  #default [10000, 14000]
        self.f_min = [500, 500]
        self.max_slope = [5, 5]

        # Position control
        self.mode = [X, Y]
        self.delta = [0, 0]
        self.direction = [0, 0]
        self.nb_pulse = [0, 0]

        # GPIO pins
        self.enable_pin = [15, 20]  # pins 10 and 38
        self.clock_pin = [14, 21]   # pins 8 and 40
        self.dir_pin = [18, 16]     # pins 12 and 36
        # self.limit_sw = [24, 23]    # pins 18 and 16

        self.init_gpio()

        # GPIO input init
        #self.gpio.set_glitch_filter(self.limit_sw[X], 50)
        #self.gpio.set_glitch_filter(self.limit_sw[Y], 50)
        #self.gpio.callback(self.limit_sw[X], pigpio.RISING_EDGE, \
        #                                   self.callback_limit_sw_x)
        #self.gpio.callback(self.limit_sw[Y], pigpio.RISING_EDGE, \
        #                                   self.callback_limit_sw_y)

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
        self.done_move.publish(self.node_name)

    # def callback_limit_sw_x(self, gpio, level, tick):
    #     self.gpio.write(self.enable_pin[X], pigpio.HIGH)
    #     print("Callback Limit Switch X")

    # def callback_limit_sw_y(self, gpio, level, tick):
    #     self.gpio.write(self.enable_pin[Y], pigpio.HIGH)
    #     print("Callback Limit Switch Y")


# Main function
if __name__ == '__main__':
    try:
        mcxy = MotorControlXY()
        mcxy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

