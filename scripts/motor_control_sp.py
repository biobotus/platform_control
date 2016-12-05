#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
from biobot_ros_msgs.msg import IntList
import pigpio
import rospy
import trajectory
from trajectory import SP

class MotorControlSP(BaseMotorControl):

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_SP is int_sp
        self.subscriber = rospy.Subscriber('Pulse_SP', IntList, self.callback_pos)

        # Frequency trapeze constants
        self.f_max     = [ 0]
        self.f_min     = [ 0]
        self.max_slope = [10]
        self.f_init    = 8000
        self.init_dir  = pigpio.HIGH
        self.init_list = []

        # Position control
        self.mode       = [SP]
        self.modes      = [SP]
        self.sync       = [1]
        self.delta      = [0]
        self.direction  = [0]
        self.nb_pulse   = [0]

        # GPIO pins
        self.enable_pin = [[27]]  # pin 15
        self.limit_sw   = [[23]]  # pin 16
        self.clock_pin  = [ 18 ]  # pin 11
        self.dir_pin    = [ 17 ]  # pin 13

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            assert len(data.data) == 2
            assert type(data.data[0]) == int
            assert type(data.data[1]) == int
            assert abs(data.data[1]) < 65536
            self.delta[SP] = data.data[1]
            self.f_max = [data.data[0]]
            self.f_min = [data.data[0]]

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for SP: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        # print('{0} freq : {1}'.format(self.node_name, self.f_max))
        # print('{0} delta : {1}.'.format(self.node_name, self.delta[SP]))

        if self.delta[SP]:
            trajectory.pos_move(self)
            self.done_module.publish(self.node_name)

    def callback_limit_sw_sp(self, gpio, level, tick):
        self.cb_sw_sp.cancel()
        if '00' in self.init_list:
            self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
            self.init_list.remove('00')
            print('sw_sp pressed')

    def set_cb_sw(self):
        if self.gpio.read(self.limit_sw[0][0]):
            self.cb_sw_sp = self.gpio.callback(self.limit_sw[0][0], \
                                               pigpio.FALLING_EDGE, \
                                               self.callback_limit_sw_sp)


# Main function
if __name__ == '__main__':
    try:
        mcsp = MotorControlSP()
        mcsp.listener()

    except rospy.ROSInterruptException as e:
        print(e)

