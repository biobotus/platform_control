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
        self.f_max      = [10000, 14000]
        self.f_min      = [500,   500  ]
        self.max_slope  = [5,     5    ]

        # Position control
        self.mode       = [X, Y]
        self.sync       = [2, 0] # Axis X has 2 motors that have to be synced
        self.delta      = [0, 0]
        self.direction  = [0, 0]
        self.nb_pulse   = [0, 0]

        # GPIO pins
        self.enable_pin     = [[6,  5],  16]  # pins 33, 31 and 36
        self.clock_pin      = [26,       21]  # pins 37     and 40
        self.dir_pin        = [[19, 13], 20]  # pins 35     and 38
        self.limit_sw_init  = [[2,  3],   4]  # 
        self.limit_sw_end   = [[7,  8],   9]  # pins 7,  11 and 15 

        self.init_gpio()

    # Callback for new position
    def callback_pos(self, data):
        try:
            self.delta = data.data
            print(self.delta)
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

        print(self.delta)
        trajectory.pos_move(self)
        self.done_move.publish(self.node_name)


# Main function
if __name__ == '__main__':
    try:
        mcxy = MotorControlXY()
        mcxy.listener()

    except rospy.ROSInterruptException as e:
        print(e)

