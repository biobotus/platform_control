#!/usr/bin/python

# Imports
from base_motor_control import BaseMotorControl
import pigpio
import rospy
from std_msgs.msg import Int32
import trajectory
from trajectory import SP

class MotorControlSP(BaseMotorControl):

    def __init__(self):
        BaseMotorControl.__init__(self)

        # ROS subscriptions
        # Expected format of Pulse_SP is [int_sp]
        self.subscriber = rospy.Subscriber('Pulse_SP', Int32, self.callback_pos)

        # Frequency trapeze constants
        self.f_max      = [20000]
        self.f_min      = [20000]
        self.max_slope  = [10]

        # Position control
        self.mode       = [SP]
        self.sync       = [0]
        self.delta      = [0]
        self.direction  = [0]
        self.nb_pulse   = [0]

        # GPIO pins
        self.enable_pin     = [13]  # pin 33
        self.clock_pin      = [26]  # pin 37
        self.dir_pin        = [19]  # pin 35
        self.limit_sw_init  = [2]   # pin 3
        self.limit_sw_end   = [4]   # pin 7

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            self.delta[SP] = data.data
            assert type(self.delta[SP]) == int
            assert abs(self.delta[SP]) < 65536

        except (SyntaxError, AssertionError) as e:
            msg = 'Invalid pulse number received for SP: {0}'.format(e)
            print(msg)
            self.error.publish('[code, {0}]'.format(self.node_name))  # TODO
            return

        trajectory.pos_move(self)
        self.done_move.publish(self.node_name)


# Main function
if __name__ == '__main__':
    try:
        mcsp = MotorControlSP()
        mcsp.listener()

    except rospy.ROSInterruptException as e:
        print(e)

