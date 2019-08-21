#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: multiple_leg_pair_state_machine.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 10/10/2017
#
# Description:
# Simple state machine controlling the position and movements of multiple_leg_pair_state_machine
# connected pairs of 2-dof robotic limbs
#
#                              (Single Pair)
#-------------------------------------------------------------------------
#                                    |
#                                    |      |===|          \ |===|
#    o_|===|_o   ->  __o_|===|_o     |   -=o|===|o-=-  ->   o|===|o-=--
#   /         \                  \   |      |===|            |===|
#              (front)               |                (top)
#
#
#                             (Multiple Pairs)
#
#                      |===|                \ |===|
#                   -=o|===|o-=-             o|===|o-=--
#                      |===|                  |===|
#                        |          ->          |
#                      |===|                  |===| /
#                   -=o|===|o-=-         --=-o|===|o
#                      |===|                  |===|
#                                  (top)
#
# The easiest way to support turns would be to add another z-axis revolute joint
# at the linkage between each segment.
'''

import rospy
from smach import State, StateMachine, Concurrence
import smach_ros

from std_msgs.msg import Float32, String
import time, math

class MoveLeg(State):
    def __init__(self, knee_target_pos, hip_target_pos, topic_prefix, knee_pub_topic, knee_sub_topic, hip_pub_topic, hip_sub_topic, rate, side):
        State.__init__(self, outcomes=['success', 'failure'])
        self.side = side
        self.knee_current_pos = 0.0  # initial values
        self.hip_current_pos = 0.0  # initial values
        if (side == 'right'):
            self.knee_target_pos = knee_target_pos
            self.hip_target_pos = hip_target_pos
        else:  # side == 'left'
            self.knee_target_pos = -knee_target_pos
            self.hip_target_pos = -hip_target_pos

        # ROS stuff
        self.topic_prefix = topic_prefix
        self.knee_pub_topic = knee_pub_topic
        self.knee_sub_topic = knee_sub_topic
        self.hip_pub_topic = hip_pub_topic
        self.hip_sub_topic = hip_sub_topic
        self.knee_pub = rospy.Publisher(self.topic_prefix + '/' + self.knee_pub_topic, Float32, queue_size=1)
        self.hip_pub = rospy.Publisher(self.topic_prefix + '/' + self.hip_pub_topic, Float32, queue_size=1)
        self.rate = rospy.Rate(rate)

    def execute(self, userdata):
        knee_sub = rospy.Subscriber(self.topic_prefix + '/' + self.knee_sub_topic, Float32, self.knee_callback)
        hip_sub = rospy.Subscriber(self.topic_prefix  + '/' + self.hip_sub_topic, Float32, self.hip_callback)

        timeout_counter = time.time()
        timeout_time = 10.0

        settle_tolerance = 0.08  # < 5 degrees
        settle_time = 0.5
        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False
        while True:
            # Knee success condition
            if (abs(self.knee_current_pos - self.knee_target_pos) < settle_tolerance):
                if not knee_settle_countdown:
                    knee_settle_countdown = True
                    knee_settle_start = time.time()
                elif ((time.time() - knee_settle_start) > settle_time):
                    knee_done = True

            # Hip success condition
            if (abs(self.hip_current_pos - self.hip_target_pos) < settle_tolerance):
                if not hip_settle_countdown:
                    hip_settle_countdown = True
                    hip_settle_start = time.time()
                elif ((time.time() - hip_settle_start) > settle_time):
                    hip_done = True

            if ((time.time() - timeout_counter) > timeout_time):
                return 'failure'
            elif (knee_done and hip_done):
                return 'success'
            else:
                # TODO: Add a position/effort ramp function
                self.knee_pub.publish(self.knee_target_pos)
                self.hip_pub.publish(self.hip_target_pos)
                self.rate.sleep()

    def knee_callback(self, msg):
        self.knee_current_pos = msg.data

    def hip_callback(self, msg):
        self.hip_current_pos = msg.data

def main():
    # ROS stuff - may move into MoveLeg class for further composability
    rospy.init_node('leg_pair_state_machine', anonymous=True)

    # TODO: replace these monolithic blocks with dictionaries and for-loops
    #### LEG STATE INSTANCES ####
    ## centipede segment 0
    # left leg State instances
    seg_0_left_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_0_left_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_0_left_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_0_left_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_0_left_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_0_left_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_0', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')

    # right leg State instances
    seg_0_right_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_0_right_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_0_right_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_0_right_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_0_right_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_0_right_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_0', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')

    ## centipede segment 1
    # left leg State instances
    seg_1_left_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_1_left_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_1_left_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_1_left_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_1_left_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_1_left_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')

    # right leg State instances
    seg_1_right_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_1_right_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_1_right_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_1_right_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_1_right_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_1_right_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')

    ## centipede segment 2
    # left leg State instances
    seg_2_left_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_2_left_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_2_left_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_2_left_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_2_left_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    seg_2_left_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_2', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')

    # right leg State instances
    seg_2_right_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_2_right_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_2_right_lift_forward = MoveLeg(-1.57079633, 0.261799, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_2_right_plant_forward = MoveLeg(-0.34906585, 0.261799, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_2_right_plant_backward = MoveLeg(-0.34906585, -0.261799, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    seg_2_right_lift_backward = MoveLeg(-1.57079633, -0.261799, 'centipede_segment_2', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')

    # add more segment-specific leg instances here...

    # TODO: replace these monolithic blocks with dictionaries and for-loops
    #### CONCURRENT LEG INSTANCES ####
    ## centipede_segment_0
    seg_0_sm_con_hold = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_HOLD':'success'}})
    with seg_0_sm_con_hold:
        Concurrence.add('LEFT_HOLD', seg_0_left_hold)
        Concurrence.add('RIGHT_HOLD', seg_0_right_hold)
    seg_0_sm_con_right_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_LIFT':'success'}})
    with seg_0_sm_con_right_lift_start:
        Concurrence.add('LEFT_HOLD', seg_0_left_hold)
        Concurrence.add('RIGHT_LIFT', seg_0_right_lift)
    seg_0_sm_con_left_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT':'success', 'RIGHT_HOLD':'success'}})
    with seg_0_sm_con_left_lift_start:
        Concurrence.add('LEFT_LIFT', seg_0_left_lift)
        Concurrence.add('RIGHT_HOLD', seg_0_right_hold)
    seg_0_sm_con_right_push = Concurrence(outcomes=['success', 'failure'],default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success'}})
    with seg_0_sm_con_right_push:
        Concurrence.add('LEFT_LIFT_FORWARD', seg_0_left_lift_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_0_right_plant_backward)
    seg_0_sm_con_left_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success'}})
    with seg_0_sm_con_left_plant:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_0_left_plant_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_0_right_plant_backward)
    seg_0_sm_con_right_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_LIFT_BACKWARD':'success'}})
    with seg_0_sm_con_right_lift:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_0_left_plant_forward)
        Concurrence.add('RIGHT_LIFT_BACKWARD', seg_0_right_lift_backward)
    seg_0_sm_con_left_push = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_LIFT_FORWARD':'success'}})
    with seg_0_sm_con_left_push:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_0_left_plant_backward)
        Concurrence.add('RIGHT_LIFT_FORWARD', seg_0_right_lift_forward)
    seg_0_sm_con_right_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_0_sm_con_right_plant:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_0_left_plant_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_0_right_plant_forward)
    seg_0_sm_con_left_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_0_sm_con_left_lift:
        Concurrence.add('LEFT_LIFT_BACKWARD', seg_0_left_lift_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_0_right_plant_forward)

    ## centipede_segment_1
    seg_1_sm_con_hold = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_HOLD':'success'}})
    with seg_1_sm_con_hold:
        Concurrence.add('LEFT_HOLD', seg_1_left_hold)
        Concurrence.add('RIGHT_HOLD', seg_1_right_hold)
    seg_1_sm_con_right_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_LIFT':'success'}})
    with seg_1_sm_con_right_lift_start:
        Concurrence.add('LEFT_HOLD', seg_1_left_hold)
        Concurrence.add('RIGHT_LIFT', seg_1_right_lift)
    seg_1_sm_con_left_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT':'success', 'RIGHT_HOLD':'success'}})
    with seg_1_sm_con_left_lift_start:
        Concurrence.add('LEFT_LIFT', seg_1_left_lift)
        Concurrence.add('RIGHT_HOLD', seg_1_right_hold)
    seg_1_sm_con_right_push = Concurrence(outcomes=['success', 'failure'],default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success'}})
    with seg_1_sm_con_right_push:
        Concurrence.add('LEFT_LIFT_FORWARD', seg_1_left_lift_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_1_right_plant_backward)
    seg_1_sm_con_left_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success' }})
    with seg_1_sm_con_left_plant:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_1_left_plant_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_1_right_plant_backward)
    seg_1_sm_con_right_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_LIFT_BACKWARD':'success'}})
    with seg_1_sm_con_right_lift:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_1_left_plant_forward)
        Concurrence.add('RIGHT_LIFT_BACKWARD', seg_1_right_lift_backward)
    seg_1_sm_con_left_push = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_LIFT_FORWARD':'success'}})
    with seg_1_sm_con_left_push:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_1_left_plant_backward)
        Concurrence.add('RIGHT_LIFT_FORWARD', seg_1_right_lift_forward)
    seg_1_sm_con_right_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_1_sm_con_right_plant:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_1_left_plant_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_1_right_plant_forward)
    seg_1_sm_con_left_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_1_sm_con_left_lift:
        Concurrence.add('LEFT_LIFT_BACKWARD', seg_1_left_lift_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_1_right_plant_forward)

    ## centipede_segment_2
    seg_2_sm_con_hold = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_HOLD':'success'}})
    with seg_2_sm_con_hold:
        Concurrence.add('LEFT_HOLD', seg_2_left_hold)
        Concurrence.add('RIGHT_HOLD', seg_2_right_hold)
    seg_2_sm_con_right_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_HOLD':'success', 'RIGHT_LIFT':'success'}})
    with seg_2_sm_con_right_lift_start:
        Concurrence.add('LEFT_HOLD', seg_2_left_hold)
        Concurrence.add('RIGHT_LIFT', seg_2_right_lift)
    seg_2_sm_con_left_lift_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT':'success', 'RIGHT_HOLD':'success'}})
    with seg_2_sm_con_left_lift_start:
        Concurrence.add('LEFT_LIFT', seg_2_left_lift)
        Concurrence.add('RIGHT_HOLD', seg_2_right_hold)
    seg_2_sm_con_right_push = Concurrence(outcomes=['success', 'failure'],default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success'}})
    with seg_2_sm_con_right_push:
        Concurrence.add('LEFT_LIFT_FORWARD', seg_2_left_lift_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_2_right_plant_backward)
    seg_2_sm_con_left_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_PLANT_BACKWARD':'success' }})
    with seg_2_sm_con_left_plant:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_2_left_plant_forward)
        Concurrence.add('RIGHT_PLANT_BACKWARD', seg_2_right_plant_backward)
    seg_2_sm_con_right_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_FORWARD':'success', 'RIGHT_LIFT_BACKWARD':'success'}})
    with seg_2_sm_con_right_lift:
        Concurrence.add('LEFT_PLANT_FORWARD', seg_2_left_plant_forward)
        Concurrence.add('RIGHT_LIFT_BACKWARD', seg_2_right_lift_backward)
    seg_2_sm_con_left_push = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_LIFT_FORWARD':'success'}})
    with seg_2_sm_con_left_push:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_2_left_plant_backward)
        Concurrence.add('RIGHT_LIFT_FORWARD', seg_2_right_lift_forward)
    seg_2_sm_con_right_plant = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_PLANT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_2_sm_con_right_plant:
        Concurrence.add('LEFT_PLANT_BACKWARD', seg_2_left_plant_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_2_right_plant_forward)
    seg_2_sm_con_left_lift = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'LEFT_LIFT_BACKWARD':'success', 'RIGHT_PLANT_FORWARD':'success'}})
    with seg_2_sm_con_left_lift:
        Concurrence.add('LEFT_LIFT_BACKWARD', seg_2_left_lift_backward)
        Concurrence.add('RIGHT_PLANT_FORWARD', seg_2_right_plant_forward)

    # add more segment-specific concurrent leg pair instances here...

    # TODO: replace these monolithic blocks with dictionaries and for-loops
    #### CONCURRENT SEGMENT INSTANCES ####
    centipede_sm_con_hold = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_HOLD':'success', 'SEG_1_HOLD':'success', 'SEG_2_HOLD':'success'}})
    with centipede_sm_con_hold:
        # even segments
        Concurrence.add('SEG_0_HOLD', seg_0_sm_con_hold)
        Concurrence.add('SEG_2_HOLD', seg_2_sm_con_hold)
        # odd segments
        Concurrence.add('SEG_1_HOLD', seg_1_sm_con_hold)
        # add more concurrent leg instances here...

    centipede_sm_con_walk_start = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_LEFT_LIFT_START':'success', 'SEG_1_LEFT_LIFT_START':'success', 'SEG_2_LEFT_LIFT_START':'success'}})
    with centipede_sm_con_walk_start:
        # even segments
        Concurrence.add('SEG_0_LEFT_LIFT_START', seg_0_sm_con_left_lift_start)
        Concurrence.add('SEG_2_LEFT_LIFT_START', seg_2_sm_con_left_lift_start)
        # odd segments
        Concurrence.add('SEG_1_LEFT_LIFT_START', seg_1_sm_con_right_lift_start)

    centipede_sm_con_walk_step_0 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_RIGHT_PUSH':'success', 'SEG_1_LEFT_PUSH':'success', 'SEG_2_RIGHT_PUSH':'success'}})
    with centipede_sm_con_walk_step_0:
        # even segments
        Concurrence.add('SEG_0_RIGHT_PUSH', seg_0_sm_con_right_push)
        Concurrence.add('SEG_2_RIGHT_PUSH', seg_2_sm_con_right_push)
        # odd segments
        Concurrence.add('SEG_1_LEFT_PUSH', seg_1_sm_con_left_push)

    centipede_sm_con_walk_step_1 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_LEFT_PLANT':'success', 'SEG_1_RIGHT_PLANT':'success', 'SEG_2_LEFT_PLANT':'success'}})
    with centipede_sm_con_walk_step_1:
        # even segments
        Concurrence.add('SEG_0_LEFT_PLANT', seg_0_sm_con_left_plant)  # FIXME: Don't worry about it now but need to add more concurrent leg instances to support reversing/turns
        Concurrence.add('SEG_2_LEFT_PLANT', seg_2_sm_con_left_plant)
        # odd segments
        Concurrence.add('SEG_1_RIGHT_PLANT', seg_1_sm_con_right_plant)

    centipede_sm_con_walk_step_2 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_RIGHT_LIFT':'success', 'SEG_1_LEFT_LIFT':'success', 'SEG_2_RIGHT_LIFT':'success'}})
    with centipede_sm_con_walk_step_2:
        # even segments
        Concurrence.add('SEG_0_RIGHT_LIFT', seg_0_sm_con_right_lift)
        Concurrence.add('SEG_2_RIGHT_LIFT', seg_2_sm_con_right_lift)
        # odd segments
        Concurrence.add('SEG_1_LEFT_LIFT', seg_1_sm_con_left_lift)

    centipede_sm_con_walk_step_3 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_LEFT_PUSH':'success', 'SEG_1_RIGHT_PUSH':'success', 'SEG_2_LEFT_PUSH':'success'}})
    with centipede_sm_con_walk_step_3:
        # even segments
        Concurrence.add('SEG_0_LEFT_PUSH', seg_0_sm_con_left_push)
        Concurrence.add('SEG_2_LEFT_PUSH', seg_2_sm_con_left_push)
        # odd segments
        Concurrence.add('SEG_1_RIGHT_PUSH', seg_1_sm_con_right_push)

    centipede_sm_con_walk_step_4 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_RIGHT_PLANT':'success', 'SEG_1_LEFT_PLANT':'success', 'SEG_2_RIGHT_PLANT':'success'}})
    with centipede_sm_con_walk_step_4:
        # even segments
        Concurrence.add('SEG_0_RIGHT_PLANT', seg_0_sm_con_right_plant)
        Concurrence.add('SEG_2_RIGHT_PLANT', seg_2_sm_con_right_plant)
        # odd segments
        Concurrence.add('SEG_1_LEFT_PLANT', seg_1_sm_con_left_plant)

    centipede_sm_con_walk_step_5 = Concurrence(outcomes=['success', 'failure'], default_outcome='failure', outcome_map={'success':{'SEG_0_LEFT_LIFT':'success', 'SEG_1_RIGHT_LIFT':'success', 'SEG_2_LEFT_LIFT':'success'}})
    with centipede_sm_con_walk_step_5:
        # even segments
        Concurrence.add('SEG_0_LEFT_LIFT', seg_0_sm_con_left_lift)
        Concurrence.add('SEG_2_LEFT_LIFT', seg_2_sm_con_left_lift)
        # odd segments
        Concurrence.add('SEG_1_RIGHT_LIFT', seg_1_sm_con_right_lift)

    # add more concurrent segment instances here...

    centipede_sm_walk_forward = StateMachine(outcomes=['success'])

    # Open centipede_sm_walk_forward
    with centipede_sm_walk_forward:
        # Add these concurrent centipede container instances to the top-level StateMachine centipede_sm_walk_forward
        StateMachine.add('HOLD', centipede_sm_con_hold, transitions={'failure':'HOLD', 'success':'WALK_START'})
        StateMachine.add('WALK_START', centipede_sm_con_walk_start, transitions={'failure':'WALK_START', 'success':'WALK_STEP_0'})
        StateMachine.add('WALK_STEP_0', centipede_sm_con_walk_step_0, transitions={'failure':'WALK_STEP_0', 'success':'WALK_STEP_1'})
        StateMachine.add('WALK_STEP_1', centipede_sm_con_walk_step_1, transitions={'failure':'WALK_STEP_1', 'success':'WALK_STEP_2'})
        StateMachine.add('WALK_STEP_2', centipede_sm_con_walk_step_2, transitions={'failure':'WALK_STEP_2', 'success':'WALK_STEP_3'})
        StateMachine.add('WALK_STEP_3', centipede_sm_con_walk_step_3, transitions={'failure':'WALK_STEP_3', 'success':'WALK_STEP_4'})
        StateMachine.add('WALK_STEP_4', centipede_sm_con_walk_step_4, transitions={'failure':'WALK_STEP_4', 'success':'WALK_STEP_5'})
        StateMachine.add('WALK_STEP_5', centipede_sm_con_walk_step_5, transitions={'failure':'WALK_STEP_5', 'success':'WALK_STEP_0'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('simple_state_machine_user_input', centipede_sm_walk_forward, '/SM_ROOT')
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing 'centipede_sm_walk_forward'\n")

    # Execute the state machine
    print("Input received. Executing 'centipede_sm_walk_forward'...")
    centipede_sm_walk_forward.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
