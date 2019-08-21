#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: centipede_odd_even_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/13/2017
# Edit Date: 11/29/2017
#
# Description:
# Finite state machine controlling the position and movements of
# multiple segments
'''

import sys
import argparse
import rospy
from std_msgs.msg import Float32, String
from smach import State, StateMachine, Concurrence
import smach_ros


class Step(State):
    def __init__(self,
                 segment_pub_topic, action_pub_msg,                                    # NOTE: Again, I'm experimenting with different State configurations /
                 segment_sub_topic, success_segment_sub_msg, failure_segment_sub_msg,  #       consistency is a low priority for me right now...
                 ros_rate, state_name):
        State.__init__(self, outcomes=['success', 'failure'])
        # ROS stuff
        self.segment_pub = rospy.Publisher(segment_pub_topic, String, queue_size=1)
        self.action_pub_msg = action_pub_msg
        self.segment_sub = rospy.Subscriber(segment_sub_topic, String, self.segment_callback)
        self.success_segment_sub_msg = success_segment_sub_msg
        self.failure_segment_sub_msg = failure_segment_sub_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name

        self.active_flag = False
        self.received_sub_msg = None

    def execute(self, userdata):
        self.active_flag = True

        # Send a message down the chain
        if self.action_pub_msg is not None:
            print("State: " + self.state_name + ", Sent sub_msg: " + self.action_pub_msg)
            self.segment_pub.publish(self.action_pub_msg)

        if (self.success_segment_sub_msg is None and self.failure_segment_sub_msg is None):
            return 'success'
        else:  # wait for a response to come back up chain - could add timeout here
            while not rospy.is_shutdown():
                if self.received_sub_msg is not None:
                    if (self.received_sub_msg == self.success_segment_sub_msg):
                        self.clean_up()
                        return 'success'
                    elif (self.received_sub_msg == self.failure_segment_sub_msg):
                        self.clean_up()
                        return 'failure'
                    else:
                        print("Unrecognized sub_msg: " + self.received_sub_msg)
                    self.received_sub_msg = None
                self.rate.sleep()
            return 'failure'  # if rospy.is_shutdown()

    def segment_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ", Received sub_msg: " + msg.data)
            self.received_sub_msg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.received_sub_msg = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    # For additional argument checking, use this method: https://stackoverflow.com/a/14117511
    parser.add_argument('-ros_rate', type=float, default=100.0, help="type=float, default=100.0, Description='rate at which ROS node publishes")
    parser.add_argument('-num_segments', type=int, default=2, help="type=int, default=2, Description='number of segments'")
    parser.add_argument('-from_segment_0', type=str, required=True, help="type=str, Description='segment ROS topic to subscribe to'")
    parser.add_argument('-to_segment_0', type=str, required=True, help="type=str, Description='segment ROS topic to publish to'")
    parser.add_argument('-from_segment_1', type=str, required=True, help="type=str, Description='segment ROS topic to subscribe to'")
    parser.add_argument('-ts1', '--to_segment_1', type=str, required=True, help="type=str, Description='segment ROS topic to publish to'")
    max_segments = 20
    for i in range(2, max_segments, 1):
        parser.add_argument('-from_segment_'+str(i), type=str, help="type=str, Description='segment ROS topic to subscribe to'")
        parser.add_argument('-to_segment_'+str(i), type=str, help="type=str, Description='segment ROS topic to publish to'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    rospy.init_node('centipede_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    args_dict = vars(parser)
    num_segments = parser.num_segments
    ros_rate = parser.ros_rate
    seg_to_centipede_dict = {}
    centipede_to_seg_dict = {}
    for i in range(num_segments):
        seg_to_centipede_dict['seg' + str(i)] = args_dict['from_segment_'+str(i)]
        centipede_to_seg_dict['seg' + str(i)] = args_dict['to_segment_'+str(i)]

    # Step State instances
    # Start
    start_state_dict = {}
    for i in range(num_segments):
        if i % 2 == 0:  # even segments -> StartLeft (Init + StepRight)
            start_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StartLeft",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
        else:           # odd segments -> StartRight (Init + StepLeft)
            start_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StartRight",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
    # Step A
    step_a_state_dict = {}
    for i in range(num_segments):
        if i % 2 == 0:  # even segments -> StepLeft
            step_a_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StepLeft",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
        else:           # odd segments -> StepRight
            step_a_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StepRight",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
    # Step B
    step_b_state_dict = {}
    for i in range(num_segments):
        if i % 2 == 0:  # even segments -> StepRight
            step_b_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StepRight",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
        else:           # odd segments -> StepLeft
            step_b_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StepLeft",
                seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))

    # Concurrence instances
    # Start
    step_start_con_success_dict = {}
    for state_name in start_state_dict:
        step_start_con_success_dict[state_name.upper()] = 'success'
    step_start_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
        outcome_map={'success':step_start_con_success_dict})
    with step_start_con:
        for state_name in start_state_dict:
            Concurrence.add(state_name.upper(), start_state_dict[state_name])
    # Step A
    step_a_con_success_dict = {}
    for state_name in step_a_state_dict:
        step_a_con_success_dict[state_name.upper()] = 'success'
    step_a_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
        outcome_map={'success':step_a_con_success_dict})
    with step_a_con:
        for state_name in step_a_state_dict:
            Concurrence.add(state_name.upper(), step_a_state_dict[state_name])
    # Step B
    step_b_con_success_dict = {}
    for state_name in step_b_state_dict:
        step_b_con_success_dict[state_name.upper()] = 'success'
    step_b_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
        outcome_map={'success':step_b_con_success_dict})
    with step_b_con:
        for state_name in step_b_state_dict:
            Concurrence.add(state_name.upper(), step_b_state_dict[state_name])

    # Create a SMACH state machine
    centipede_fsm_node = StateMachine(outcomes=['success'])

    # Open the SMACH state machine
    with centipede_fsm_node:
        # Add these concurrent centipede container instances to the top-level StateMachine
        StateMachine.add('START', step_start_con, transitions={'failure':'START', 'success': 'STEP_A'})
        StateMachine.add('STEP_A', step_a_con, transitions={'failure':'STEP_A', 'success': 'STEP_B'})
        StateMachine.add('STEP_B', step_b_con, transitions={'failure':'STEP_B', 'success': 'STEP_A'})

    # Create and start the introspection server - for visualization / debugging
    sis = smach_ros.IntrospectionServer('centipede_odd_even_fsm_node' + str(rospy.get_name()), centipede_fsm_node, '/SM_ROOT') #+ str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: centipede_odd_even_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: centipede_odd_even_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = centipede_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
