#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: centipede_sym_staggered_triplets_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/13/2017
# Edit Date: 10/13/2017
#
# Description:
# Finite state machine controlling the position and movements of
# multiple segments
'''

import rospy
from smach import State, StateMachine, Concurrence
import smach_ros
from std_msgs.msg import Float32, String

import time, math, sys

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
            while True:
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

    def segment_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ", Received sub_msg: " + msg.data)
            self.received_sub_msg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.received_sub_msg = None

def main():
    rospy.init_node('centipede_fsm_node', anonymous=True)

    centipede_to_seg_dict = {}
    seg_to_centipede_dict = {}
    num_segments = None
    ros_rate = None
    if ( (len(sys.argv) < 6) ):
        print("usage: centipede_fsm_node.py centipede_to_segment0 segment0_to_centipede \
            centipede_to_segment1 setment1_to_centipede (etc...) num_segments ros_rate")
    else:
        print sys.stderr, sys.argv
        num_segments = int(sys.argv[-4])
        print("num_segments = " + str(num_segments))
        ros_rate = float(sys.argv[-3])
        for i in range(num_segments):
            centipede_to_seg_dict['seg' + str(i)] = sys.argv[2*i+1]
            seg_to_centipede_dict['seg' + str(i)] = sys.argv[2*i+2]

    # Step State instances
    # Start - Lift
    start_state_dict = {}
    for i in range(num_segments):
        start_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "StartLift",
            seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
    # Step A - Plant-Push
    step_a_state_dict = {}
    for i in range(num_segments):
        step_a_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "Plant",
            seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))
    # Step B - Recover
    step_b_state_dict = {}
    for i in range(num_segments):
        step_b_state_dict['seg' + str(i)] = Step(centipede_to_seg_dict['seg' + str(i)], "Recover",
            seg_to_centipede_dict['seg' + str(i)], "Ready", None, ros_rate, 'seg' + str(i))

    # Concurrence instances # NOTE: could probably make this more flexible w/ dictionaries but also makes code more opaque :/
    # Start - Lift
    start_state_1st_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                      outcome_map={'success':{'SEG0':'success', 'SEG3':'success'}})
    with start_state_1st_con:
        for i in range(0, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), start_state_dict[seg_key])

    start_state_2nd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                      outcome_map={'success':{'SEG1':'success', 'SEG4':'success'}})
    with start_state_2nd_con:
        for i in range(1, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), start_state_dict[seg_key])

    start_state_3rd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                      outcome_map={'success':{'SEG2':'success', 'SEG5':'success'}})
    with start_state_3rd_con:
        for i in range(2, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), start_state_dict[seg_key])

    # Step A - Plant-Push
    step_a_state_1st_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG0':'success', 'SEG3':'success'}})
    with step_a_state_1st_con:
        for i in range(0, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_a_state_dict[seg_key])

    step_a_state_2nd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG1':'success', 'SEG4':'success'}})
    with step_a_state_2nd_con:
        for i in range(1, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_a_state_dict[seg_key])

    step_a_state_3rd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG2':'success', 'SEG5':'success'}})
    with step_a_state_3rd_con:
        for i in range(2, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_a_state_dict[seg_key])

    # Step B - Recover
    step_b_state_1st_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG0':'success', 'SEG3':'success'}})
    with step_b_state_1st_con:
        for i in range(0, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_b_state_dict[seg_key])

    step_b_state_2nd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG1':'success', 'SEG4':'success'}})
    with step_b_state_2nd_con:
        for i in range(1, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_b_state_dict[seg_key])

    step_b_state_3rd_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                       outcome_map={'success':{'SEG2':'success', 'SEG5':'success'}})
    with step_b_state_3rd_con:
        for i in range(2, num_segments, 3):
            seg_key = 'seg' + str(i)
            Concurrence.add(seg_key.upper(), step_b_state_dict[seg_key])

    # Create a SMACH state machine
    centipede_fsm_node = StateMachine(outcomes=['success'])

    # Open the SMACH state machine
    with centipede_fsm_node:
        # Add these instances to the top-level StateMachine
        StateMachine.add('START_1ST', start_state_1st_con, transitions={'failure':'START_1ST', 'success': 'START_2ND'})
        StateMachine.add('START_2ND', start_state_2nd_con, transitions={'failure':'START_2ND', 'success': 'START_3RD'})
        StateMachine.add('START_3RD', start_state_3rd_con, transitions={'failure':'START_3RD', 'success': 'STEP_B_1ST'})

        # Main loop
        StateMachine.add('STEP_B_1ST', step_b_state_1st_con, transitions={'failure':'STEP_B_1ST', 'success': 'STEP_A_1ST'})
        StateMachine.add('STEP_A_1ST', step_a_state_1st_con, transitions={'failure':'STEP_A_1ST', 'success': 'STEP_B_2ND'})
        StateMachine.add('STEP_B_2ND', step_b_state_2nd_con, transitions={'failure':'STEP_B_2ND', 'success': 'STEP_A_2ND'})
        StateMachine.add('STEP_A_2ND', step_a_state_2nd_con, transitions={'failure':'STEP_A_2ND', 'success': 'STEP_B_3RD'})
        StateMachine.add('STEP_B_3RD', step_b_state_3rd_con, transitions={'failure':'STEP_B_3RD', 'success': 'STEP_A_3RD'})
        StateMachine.add('STEP_A_3RD', step_a_state_3rd_con, transitions={'failure':'STEP_A_3RD', 'success': 'STEP_B_1ST'})

    # Create and start the introspection server - for visualization / debugging
    sis = smach_ros.IntrospectionServer('centipede_sym_staggered_triplets_fsm_node' + str(rospy.get_name()), centipede_fsm_node, '/SM_ROOT') #+ str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: centipede_sym_staggered_triplets_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: centipede_sym_staggered_triplets_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = centipede_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
