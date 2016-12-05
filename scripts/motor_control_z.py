#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
from biobot_ros_msgs.msg import IntList
import pigpio
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
        self.f_max     = [8000, 8000, 10000]
        self.f_min     = [ 500,  500,   500]
        self.max_slope = [  7,   7,    7]
        self.f_init    = 3500
        self.init_dir  = pigpio.LOW
        self.init_list = []

        # Position control
        self.mode      = [Z0, Z1, Z2]
        self.modes     = [Z0, Z1, Z2]
        self.sync      = [1,  1,  1 ]
        self.delta     = [0,  0,  0 ]
        self.direction = [1,  1,  1 ]
        self.nb_pulse  = [0,  0,  0 ]

        # GPIO pins
        self.enable_pin = [[23], [18], [14]]  # pins [22], [23] and [33]
        self.limit_sw   = [[19], [16], [26]]  # pins [32], [36] and [38]
        self.clock_pin  = [  24 ,   27 ,  15 ]  # pins  21 ,  31  and  37
        self.dir_pin    = [ 22 ,   17 ,  4 ]  # pins  19 ,  29  and  35

        self.init_gpio()
        self.cb_sw_z0_flag = False
        self.cb_sw_z1_flag = False
        self.cb_sw_z2_flag = False
        self.cb_sw_z0_flag_init = False
        self.cb_sw_z1_flag_init = False
        self.cb_sw_z2_flag_init = False
        self.cb_sw_z0 = self.gpio.callback(self.limit_sw[0][0], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_z0)
        self.cb_sw_z1 = self.gpio.callback(self.limit_sw[1][0], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_z1)
        self.cb_sw_z2 = self.gpio.callback(self.limit_sw[2][0], \
                                           pigpio.FALLING_EDGE, \
                                           self.callback_limit_sw_z2)

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
        if self.cb_sw_z0_flag_init:
            self.cb_sw_z0_flag_init = False
            self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
            if '00' in self.init_list:
                self.init_list.remove('00')
                print('sw_z0 pressed')
        elif self.cb_sw_z0_flag:
            self.cb_sw_z0_flag = False
            self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
            print("Error: limit switch Z0 pressed!")
            self.error.publish(str({"error_code": "Hw0", "name": self.node_name}))

    def callback_limit_sw_z1(self, gpio, level, tick):
        if self.cb_sw_z1_flag_init:
            self.cb_sw_z1_flag_init = False
            self.gpio.write(self.enable_pin[1][0], pigpio.LOW)
            if '10' in self.init_list:
                self.init_list.remove('10')
                print('sw_z1 pressed')
        elif self.cb_sw_z1_flag:
            self.cb_sw_z1_flag = False
            self.gpio.write(self.enable_pin[1][0], pigpio.LOW)
            print("Error: limit switch Z1 pressed!")
            self.error.publish(str({"error_code": "Hw0", "name": self.node_name}))

    def callback_limit_sw_z2(self, gpio, level, tick):
        if self.cb_sw_z2_flag_init:
            self.cb_sw_z2_flag_init = False
            self.gpio.write(self.enable_pin[2][0], pigpio.LOW)
            if '20' in self.init_list:
                self.init_list.remove('20')
                print('sw_z2 pressed')
        elif self.cb_sw_z2_flag:
            self.cb_sw_z2_flag = False
            self.gpio.write(self.enable_pin[2][0], pigpio.LOW)
            print("Error: limit switch Z2 pressed!")
            self.error.publish(str({"error_code": "Hw0", "name": self.node_name}))

    def set_cb_sw(self, init=False):
        if init:
            if self.gpio.read(self.limit_sw[0][0]):
                self.cb_sw_z0_flag_init = True
                self.cb_sw_z0_flag = False
            if self.gpio.read(self.limit_sw[1][0]):
                self.cb_sw_z1_flag_init = True
                self.cb_sw_z1_flag = False
            if self.gpio.read(self.limit_sw[2][0]):
                self.cb_sw_z2_flag_init = True
                self.cb_sw_z2_flag = False
        else:
            self.cb_sw_z0_flag = True
            self.cb_sw_z1_flag = True
            self.cb_sw_z2_flag = True

    def move_5mm(self):
        """After initialisation, move away from switch of 5 mm"""
        self.delta = [-500, -500, -500]
        self.mode = [Z0, Z1, Z2]
        trajectory.pos_move(self)

# Main function
if __name__ == '__main__':
    try:
        mcz = MotorControlZ()
        mcz.listener()

    except rospy.ROSInterruptException as e:
        print(e)

