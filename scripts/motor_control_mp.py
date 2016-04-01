#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
import pigpio
import rospy
from std_msgs.msg import Int32
import trajectory
from trajectory import MP

class MotorControlMP(BaseMotorControl):

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_MP is int_mp
        self.subscriber = rospy.Subscriber('Pulse_MP', Int32, self.callback_pos)

        # Frequency trapeze constants
        self.f_max      = [20000]
        self.f_min      = [20000]
        self.max_slope  = [10]

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
        self.dir_pin    = [ 9  ]  # pin 21

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            self.delta[MP] = data.data
            assert type(self.delta[MP]) == int
            assert abs(self.delta[MP]) < 65536

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for MP: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        trajectory.pos_move(self)
        self.done_move.publish(self.node_name)


# Main function
if __name__ == '__main__':
    try:
        mcmp = MotorControlMP()
        mcmp.listener()

    except rospy.ROSInterruptException as e:
        print(e)

