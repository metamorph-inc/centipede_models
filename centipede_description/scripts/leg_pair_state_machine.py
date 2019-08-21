#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: leg_pair_state_machine.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/09/2017
# Edit Date: 10/09/2017
#
# Description:
# Simple state machine controlling the position and movements of a pair of
# 2-dof robotic limbs
#                                    |
#                                    |      |===|          \ |===|
#    o_|===|_o   ->  __o_|===|_o     |   -=o|===|o-=-  ->   o|===|o-=--
#   /         \                  \   |      |===|            |===|
#              (front)               |                (top)
#
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

    # left leg State instances
    left_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    left_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    left_lift_forward = MoveLeg(-1.57079633, 0.785398, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    left_plant_forward = MoveLeg(-0.34906585, 0.785398, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    left_plant_backward = MoveLeg(-0.34906585, -0.785398, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')
    left_lift_backward = MoveLeg(-1.57079633, -0.785398, 'centipede_segment_1', 'left_knee_pos_cmd', 'left_knee_pos_angle', 'left_hip_pos_cmd', 'left_hip_pos_angle', 100, 'left')

    # right leg State instance
    right_hold = MoveLeg(-0.34906585, 0.0, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    right_lift = MoveLeg(-1.57079633, 0.0, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    right_lift_forward = MoveLeg(-1.57079633, 0.785398, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    right_plant_forward = MoveLeg(-0.34906585, 0.785398, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    right_plant_backward = MoveLeg(-0.34906585, -0.785398, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')
    right_lift_backward = MoveLeg(-1.57079633, -0.785398, 'centipede_segment_1', 'right_knee_pos_cmd', 'right_knee_pos_angle', 'right_hip_pos_cmd', 'right_hip_pos_angle', 100, 'right')

    # Create a top level SMACH state machine
    sm_walk_forward = StateMachine(outcomes=['success'])

    # Open sm_walk_forward
    with sm_walk_forward:
        # Create the sub SMACH concurrent state machine
        sm_con_hold = Concurrence(outcomes=['success', 'failure'],
                                  default_outcome='failure',
                                  outcome_map={'success':
                                                {'LEFT_HOLD':'success',
                                                 'RIGHT_HOLD':'success' }})
        # Open the sub container
        with sm_con_hold:
            # Add states to the container
            Concurrence.add('LEFT_HOLD', left_hold)
            Concurrence.add('RIGHT_HOLD', right_hold)

        sm_con_left_lift_start = Concurrence(outcomes=['success', 'failure'],
                                       default_outcome='failure',
                                       outcome_map={'success':
                                                     {'LEFT_LIFT':'success',
                                                      'RIGHT_HOLD':'success' }})

        with sm_con_left_lift_start:
            Concurrence.add('LEFT_LIFT', left_lift)
            Concurrence.add('RIGHT_HOLD', right_hold)

        sm_con_right_push = Concurrence(outcomes=['success', 'failure'],
                                        default_outcome='failure',
                                        outcome_map={'success':
                                                      {'LEFT_LIFT_FORWARD':'success',
                                                       'RIGHT_PLANT_BACKWARD':'success' }})
        with sm_con_right_push:
            Concurrence.add('LEFT_LIFT_FORWARD', left_lift_forward)
            Concurrence.add('RIGHT_PLANT_BACKWARD', right_plant_backward)

        sm_con_left_plant = Concurrence(outcomes=['success', 'failure'],
                                        default_outcome='failure',
                                        outcome_map={'success':
                                                      {'LEFT_PLANT_FORWARD':'success',
                                                       'RIGHT_PLANT_BACKWARD':'success' }})
        with sm_con_left_plant:
            Concurrence.add('LEFT_PLANT_FORWARD', left_plant_forward)
            Concurrence.add('RIGHT_PLANT_BACKWARD', right_plant_backward)

        sm_con_right_lift = Concurrence(outcomes=['success', 'failure'],
                                        default_outcome='failure',
                                        outcome_map={'success':
                                                      {'LEFT_PLANT_FORWARD':'success',
                                                       'RIGHT_LIFT_BACKWARD':'success' }})
        with sm_con_right_lift:
            Concurrence.add('LEFT_PLANT_FORWARD', left_plant_forward)
            Concurrence.add('RIGHT_LIFT_BACKWARD', right_lift_backward)

        sm_con_left_push = Concurrence(outcomes=['success', 'failure'],
                                       default_outcome='failure',
                                       outcome_map={'success':
                                                     {'LEFT_PLANT_BACKWARD':'success',
                                                      'RIGHT_LIFT_FORWARD':'success' }})
        with sm_con_left_push:
            Concurrence.add('LEFT_PLANT_BACKWARD', left_plant_backward)
            Concurrence.add('RIGHT_LIFT_FORWARD', right_lift_forward)

        sm_con_right_plant = Concurrence(outcomes=['success', 'failure'],
                                         default_outcome='failure',
                                         outcome_map={'success':
                                                       {'LEFT_PLANT_BACKWARD':'success',
                                                        'RIGHT_PLANT_FORWARD':'success' }})
        with sm_con_right_plant:
            Concurrence.add('LEFT_PLANT_BACKWARD', left_plant_backward)
            Concurrence.add('RIGHT_PLANT_FORWARD', right_plant_forward)

        sm_con_left_lift = Concurrence(outcomes=['success', 'failure'],
                                       default_outcome='failure',
                                       outcome_map={'success':
                                                     {'LEFT_LIFT_BACKWARD':'success',
                                                      'RIGHT_PLANT_FORWARD':'success' }})
        with sm_con_left_lift:
            Concurrence.add('LEFT_LIFT_BACKWARD', left_lift_backward)
            Concurrence.add('RIGHT_PLANT_FORWARD', right_plant_forward)

        # Add these concurrent containers to the top-level StateMachine sm_top
        StateMachine.add('LEGS_HOLD', sm_con_hold, transitions={'failure':'LEGS_HOLD', 'success':'LEGS_LEFT_LIFT'})
        StateMachine.add('LEGS_LEFT_LIFT_START', sm_con_left_lift_start, transitions={'failure':'LEGS_LEFT_LIFT_START', 'success':'LEGS_RIGHT_PUSH'})
        StateMachine.add('LEGS_RIGHT_PUSH', sm_con_right_push, transitions={'failure':'LEGS_RIGHT_PUSH', 'success':'LEGS_LEFT_PLANT'})
        StateMachine.add('LEGS_LEFT_PLANT', sm_con_left_plant, transitions={'failure':'LEGS_LEFT_PLANT', 'success':'LEGS_RIGHT_LIFT'})
        StateMachine.add('LEGS_RIGHT_LIFT', sm_con_right_lift, transitions={'failure':'LEGS_RIGHT_LIFT', 'success':'LEGS_LEFT_PUSH'})
        StateMachine.add('LEGS_LEFT_PUSH', sm_con_left_push, transitions={'failure':'LEGS_LEFT_PUSH', 'success':'LEGS_RIGHT_PLANT'})
        StateMachine.add('LEGS_RIGHT_PLANT', sm_con_right_plant, transitions={'failure':'LEGS_RIGHT_PLANT', 'success':'LEGS_LEFT_LIFT'})
        StateMachine.add('LEGS_LEFT_LIFT', sm_con_left_lift, transitions={'failure':'LEGS_LEFT_LIFT', 'success':'LEGS_RIGHT_PUSH'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('simple_state_machine_user_input', sm_walk_forward, '/SM_ROOT')
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing 'sm_walk_forward'\n")

    # Execute the state machine
    print("Input received. Executing 'sm_walk_forward'...")
    sm_walk_forward.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
