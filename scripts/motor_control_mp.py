#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
import pigpio
import rospy
from std_msgs.msg import Int32
from platform_control.msg import IntList
import trajectory
from trajectory import MP

class MotorControlMP(BaseMotorControl):

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_MP is int_mp
        self.subscriber = rospy.Subscriber('Pulse_MP', Int32, self.callback_pos)

        # Frequency trapeze constants
        self.f_max     = [ 0]
        self.f_min     = [ 0]
        self.max_slope = [10]
        self.f_init    = 8000
        self.init_dir  = pigpio.HIGH
        self.init_list = []

        # Position control
        self.mode       = [MP]
        self.modes      = [MP]
        self.sync       = [1]
        self.delta      = [0]
        self.direction  = [0]
        self.nb_pulse   = [0]

        # GPIO pins
        self.enable_pin = [[11]]  # pin 23
        self.limit_sw   = [[21]]  # pin 40
        self.clock_pin  = [ 10 ]  # pin 19
        self.dir_pin    = [  9 ]  # pin 21

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            assert len(data.data) == 2
            assert type(self.delta[MP][0]) == int
            assert type(self.delta[MP][0]) == int
            assert abs(self.delta[MP][1]) < 65536
            assert abs(self.delta[MP][1]) < 65536
            self.delta[MP] = data.data[1]
            self.f_max = [data.data[0]]
            self.f_min = [data.data[0]]

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for MP: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        print("{0} freq : {1}".format(self.node_name, self.f_max))
        print("{0} delta : {1}.".format(self.node_name, self.delta[MP]))

        if self.delta[MP]:
            trajectory.pos_move(self)
            self.done_module.publish(self.node_name)

    def callback_limit_sw_mp(self, gpio, level, tick):
        self.gpio.write(self.enable_pin[0][0], pigpio.LOW)
        self.cb_sw_mp.cancel()
        try:
            self.init_list.remove("00")
            print("sw_mp pressed")
        except ValueError:
            pass

    def set_cb_sw(self):
        self.cb_sw_mp = self.gpio.callback(self.limit_sw[0][0], \
                                pigpio.FALLING_EDGE, self.callback_limit_sw_mp)


# Main function
if __name__ == '__main__':
    try:
        mcmp = MotorControlMP()
        mcmp.listener()

    except rospy.ROSInterruptException as e:
        print(e)

