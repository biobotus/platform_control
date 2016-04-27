#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
from biobot_ros_msgs.msg import IntList
import pigpio
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
        self.f_max     = [10000, 10000]
        self.f_min     = [  500,   500]
        self.max_slope = [    4,     5]
        self.f_init    = 1000
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
        self.enable_pin = [[24, 23], [ 7]]  # pins [18, 16] and [26]
        self.limit_sw   = [[18, 15], [14]]  # pins [12, 10] and [ 8]
        self.clock_pin  = [  8     ,  27 ]  # pins  24      and  13
        self.dir_pin    = [ 25     ,  17 ]  # pins  22      and  11

        self.init_gpio()
        self.cb_sw_x0_flag = False
        self.cb_sw_x1_flag = False
        self.cb_sw_y_flag = False
        self.cb_sw_x0 = self.gpio.callback(self.limit_sw[0][0], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_x0)
        self.cb_sw_x1 = self.gpio.callback(self.limit_sw[0][1], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_x1)
        self.cb_sw_y  = self.gpio.callback(self.limit_sw[1][0], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_y)

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
            #self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        trajectory.pos_move(self)
        self.done_module.publish(self.node_name)

    def callback_limit_sw_x0(self, gpio, level, tick):
        if self.cb_sw_x0_flag:
            self.cb_sw_x0_flag = False
            self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
            if '00' in self.init_list:
                self.init_list.remove('00')
                print('sw_x0 pressed')

    def callback_limit_sw_x1(self, gpio, level, tick):
        if self.cb_sw_x1_flag:
            self.cb_sw_x1_flag = False
            self.gpio.write(self.enable_pin[0][1], pigpio.LOW)
            if '01' in self.init_list:
                self.init_list.remove('01')
                print('sw_x1 pressed')

    def callback_limit_sw_y(self, gpio, level, tick):
        if self.cb_sw_y_flag:
            self.cb_sw_y_flag = False
            self.gpio.write(self.enable_pin[1][0], pigpio.LOW)
            if '10' in self.init_list:
                self.init_list.remove('10')
                print('sw_y pressed')

    def set_cb_sw(self):
        if self.gpio.read(self.limit_sw[0][0]):
            self.cb_sw_x0_flag = True
        if self.gpio.read(self.limit_sw[0][1]):
            self.cb_sw_x1_flag = True
        if self.gpio.read(self.limit_sw[1][0]):
            self.cb_sw_y_flag = True


# Main function
if __name__ == '__main__':
    try:
        mcxy = MotorControlXY()
        mcxy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

