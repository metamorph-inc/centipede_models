#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: centipede_tripod_direct_leg_ctrl_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 11/08/2017
# Edit Date: 11/08/2017
#
# Description:
# Finite state machine controlling the position and movements of
# multiple segments
'''

import time
import math
import argparse

import rospy
from smach import State, StateMachine, Concurrence
import smach_ros
from std_msgs.msg import Float32, String

class MoveLeg(State):
    def __init__(self,
                pub_to_leg_topic, pub_to_leg_msg,
                sub_to_leg_topic, success_leg_msg, failure_leg_msg,
                ros_rate, state_name):
       State.__init__(self, outcomes=['success', 'failure'])
       # ROS stuff
       self.leg_pub = rospy.Publisher(pub_to_leg_topic, String, queue_size=1)
       self.pub_to_leg_msg = pub_to_leg_msg
       self.leg_sub = rospy.Subscriber(sub_to_leg_topic, String, self.leg_callback)
       self.success_leg_msg = success_leg_msg
       self.failure_leg_msg = failure_leg_msg
       self.rate = rospy.Rate(ros_rate)
       self.state_name = state_name
       self.active_flag = False
       self.msg_from_leg = None

    def execute(self, userdata):
        self.active_flag = True

        # Send a message down the chain
        if self.pub_to_leg_msg is not None:
            print("State: " + self.state_name + ", Sent pub_to_leg_msg: " + self.pub_to_leg_msg)
            self.leg_pub.publish(self.pub_to_leg_msg)

        if (self.success_leg_msg is None and self.failure_leg_msg is None):
            return 'success'
        else:  # wait for a response to come back up chain - could add timeout here
            while True:
                if self.msg_from_leg is not None:
                    if (self.msg_from_leg == self.success_leg_msg):
                        self.clean_up()
                        return 'success'
                    elif (self.msg_from_leg == self.failure_leg_msg):
                        self.clean_up()
                        return 'failure'
                    else:
                        print("Unrecognized msg_from_leg: " + self.msg_from_leg)
                    self.msg_from_leg = None
                self.rate.sleep()

    def leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ", Received msg_from_leg: " + msg.data)
            self.msg_from_leg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.msg_from_leg = None

def main():
    # ROS stuff
    rospy.init_node('centipede_tripod_direct_leg_ctrl_fsm_node', anonymous=True)

    #TODO: Add some unittests
    #TODO: Place this in its own function
    parser = argparse.ArgumentParser()
    parser.add_argument('-rr', '--ros_rate', type=float, default=100.0, help="type=float, default=100.0, Description='rate at which ROS node publishes'")
    parser.add_argument('-f0l', '--from_seg0_left', type=str, required=True, help="Description='segment0 left leg ROS topic to subscribe to'")
    parser.add_argument('-t0l', '--to_seg0_left', type=str, required=True, help="Description='segment0 left leg ROS topic to publish to'")
    parser.add_argument('-f0r', '--from_seg0_right', type=str, required=True, help="Description='segment0 right leg ROS topic to subscribe to'")
    parser.add_argument('-t0r', '--to_seg0_right', type=str, required=True, help="Description='segment0 right leg ROS topic to publish to'")
    parser.add_argument('-f1l', '--from_seg1_left', type=str, required=True, help="Description='segment1  left leg ROS topic to subscribe to'")
    parser.add_argument('-t1l', '--to_seg1_left', type=str, required=True, help="Description='segment1 left leg ROS topic to publish to'")
    parser.add_argument('-f1r', '--from_seg1_right', type=str, required=True, help="Description='segment1 right leg ROS topic to subscribe to'")
    parser.add_argument('-t1r', '--to_seg1_right', type=str, required=True, help="Description='segment1 right leg ROS topic to publish to'")
    parser.add_argument('-f2l', '--from_seg2_left', type=str, required=True, help="Description='segment2  left leg ROS topic to subscribe to'")
    parser.add_argument('-t2l', '--to_seg2_left', type=str, required=True, help="Description='segment2 left leg ROS topic to publish to'")
    parser.add_argument('-f2r', '--from_seg2_right', type=str, required=True, help="Description='segment2 right leg ROS topic to subscribe to'")
    parser.add_argument('-t2r', '--to_seg2_right', type=str, required=True, help="Description='segment2 right leg ROS topic to publish to'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    #TODO: Add support for variable # of legs & num_segments arg
    args = parser.parse_args()
    args_dict = vars(args)

    num_segments = 3  #TODO: Obtain value from cmd line arg
    segment_name_list = ['seg'+str(i) for i in range(num_segments)]
    ros_rate = args.ros_rate
    leg_to_cent_dict = {}
    cent_to_leg_dict = {}
    for segment_name in segment_name_list:
        leg_to_cent_segment_dict = {'left':args_dict['from_'+ segment_name +'_left'],
                                    'right':args_dict['from_'+ segment_name +'_right']}
        cent_to_leg_segment_dict = {'left':args_dict['to_'+ segment_name +'_left'],
                                    'right':args_dict['to_'+ segment_name +'_right']}
        leg_to_cent_dict[segment_name] = leg_to_cent_segment_dict
        cent_to_leg_dict[segment_name] = cent_to_leg_segment_dict

    # State instance dictionary  #NOTE: Maybe reorder this to state_inst_dict[segment][side][action]
    pub_to_leg_msg_list = ['stand_up', 'reset',
                           'lift', 'lift_high',
                           'plant_north_plus_25', 'plant_north_minus_25',
                           'push_west_from_north_plus_25', 'push_east_from_north_minus_25']
    state_inst_dict = {}
    for msg in pub_to_leg_msg_list:
        msg_dict = {}
        for segment_name in segment_name_list:
            segment_dict = {}
            for side_name in ['left', 'right']:
                segment_dict[side_name] = MoveLeg(cent_to_leg_dict[segment_name][side_name], msg,
                                                  leg_to_cent_dict[segment_name][side_name], 'done',
                                                  None, 100, segment_name + ': '+side_name+': '+ msg)
            msg_dict[segment_name] = segment_dict
        state_inst_dict[msg] = msg_dict

    # Concurrent instances
    # stand_up
    stand_up_con_success_dict = {}
    for seg in state_inst_dict['stand_up']:
        for side in state_inst_dict['stand_up'][seg]:
            stand_up_con_success_dict[seg.upper() + side.upper()] = 'success'
    stand_up_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                               outcome_map={'success':stand_up_con_success_dict})
    with stand_up_con:
        for seg in state_inst_dict['stand_up']:
            for side in state_inst_dict['stand_up'][seg]:
                Concurrence.add(seg.upper() + side.upper(), state_inst_dict['stand_up'][seg][side])

    #TODO: Refactor for variable segment #s
    # step_a
    step_a_lift_con_success_dict = {'SEG0LEFT':'success',
                                    'SEG1RIGHT':'success',
                                    'SEG2LEFT':'success'}

    step_a_lift_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_a_lift_con_success_dict})
    with step_a_lift_con:
        Concurrence.add('SEG0LEFT', state_inst_dict['lift_high']['seg0']['left'])
        Concurrence.add('SEG1RIGHT', state_inst_dict['lift_high']['seg1']['right'])
        Concurrence.add('SEG2LEFT', state_inst_dict['lift_high']['seg2']['left'])

    step_a_plant_con_success_dict = {'SEG0LEFT':'success',
                                     'SEG1RIGHT':'success',
                                     'SEG2LEFT':'success'}

    step_a_plant_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_a_plant_con_success_dict})
    with step_a_plant_con:
        Concurrence.add('SEG0LEFT', state_inst_dict['plant_north_plus_25']['seg0']['left'])
        Concurrence.add('SEG1RIGHT', state_inst_dict['plant_north_minus_25']['seg1']['right'])
        Concurrence.add('SEG2LEFT', state_inst_dict['plant_north_plus_25']['seg2']['left'])


    step_a_push_con_success_dict = {'SEG0LEFT':'success',
                                    'SEG1RIGHT':'success',
                                    'SEG2LEFT':'success'}

    step_a_push_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_a_push_con_success_dict})
    with step_a_push_con:
        Concurrence.add('SEG0LEFT', state_inst_dict['push_west_from_north_plus_25']['seg0']['left'])
        Concurrence.add('SEG1RIGHT', state_inst_dict['push_east_from_north_minus_25']['seg1']['right'])
        Concurrence.add('SEG2LEFT', state_inst_dict['push_west_from_north_plus_25']['seg2']['left'])

    # step_b
    step_b_lift_con_success_dict = {'SEG0RIGHT':'success',
                                    'SEG1LEFT':'success',
                                    'SEG2RIGHT':'success'}

    step_b_lift_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_b_lift_con_success_dict})
    with step_b_lift_con:
        Concurrence.add('SEG0RIGHT', state_inst_dict['lift_high']['seg0']['right'])
        Concurrence.add('SEG1LEFT', state_inst_dict['lift_high']['seg1']['left'])
        Concurrence.add('SEG2RIGHT', state_inst_dict['lift_high']['seg2']['right'])

    step_b_plant_con_success_dict = {'SEG0RIGHT':'success',
                                     'SEG1LEFT':'success',
                                     'SEG2RIGHT':'success'}

    step_b_plant_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_b_plant_con_success_dict})
    with step_b_plant_con:
        Concurrence.add('SEG0RIGHT', state_inst_dict['plant_north_minus_25']['seg0']['right'])
        Concurrence.add('SEG1LEFT', state_inst_dict['plant_north_plus_25']['seg1']['left'])
        Concurrence.add('SEG2RIGHT', state_inst_dict['plant_north_minus_25']['seg2']['right'])


    step_b_push_con_success_dict = {'SEG0RIGHT':'success',
                                    'SEG1LEFT':'success',
                                    'SEG2RIGHT':'success'}

    step_b_push_con = Concurrence(outcomes=['success', 'failure'], default_outcome='failure',
                                  outcome_map={'success':step_b_push_con_success_dict})
    with step_b_push_con:
        Concurrence.add('SEG0RIGHT', state_inst_dict['push_east_from_north_minus_25']['seg0']['right'])
        Concurrence.add('SEG1LEFT', state_inst_dict['push_west_from_north_plus_25']['seg1']['left'])
        Concurrence.add('SEG2RIGHT', state_inst_dict['push_east_from_north_minus_25']['seg2']['right'])

    # Create a SMACH state machine
    centipede_fsm_node = StateMachine(outcomes=['success'])

    # Open the SMACH state machine
    with centipede_fsm_node:
        # Add these concurrent centipede container instances to the top-level StateMachine
        StateMachine.add('STAND_UP', stand_up_con, transitions={'failure':'STAND_UP', 'success':'STEP_A_LIFT'})
        StateMachine.add('STEP_A_LIFT', step_a_lift_con, transitions={'failure':'STEP_A_LIFT', 'success':'STEP_A_PLANT'})
        StateMachine.add('STEP_A_PLANT', step_a_plant_con, transitions={'failure':'STEP_A_PLANT', 'success':'STEP_B_LIFT'})
        StateMachine.add('STEP_B_LIFT', step_b_lift_con, transitions={'failure':'STEP_B_LIFT', 'success':'STEP_A_PUSH'})
        StateMachine.add('STEP_A_PUSH', step_a_push_con, transitions={'failure':'STEP_A_PUSH', 'success':'STEP_B_PLANT'})
        StateMachine.add('STEP_B_PLANT', step_b_plant_con, transitions={'failure':'STEP_B_PLANT', 'success':'STEP_B_PUSH'})
        StateMachine.add('STEP_B_PUSH', step_b_push_con, transitions={'failure':'STEP_B_PUSH', 'success':'STEP_A_LIFT'})  # https://www.youtube.com/watch?v=GibiNy4d4gc

    # Create and start the introspection server - for visualization / debugging
    sis = smach_ros.IntrospectionServer('centipede_tripod_direct_leg_ctrl_fsm_node' + str(rospy.get_name()), centipede_fsm_node, '/SM_ROOT') #+ str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: centipede_tripod_direct_leg_ctrl_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: centipede_tripod_direct_leg_ctrl_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = centipede_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()






































        # spacer
