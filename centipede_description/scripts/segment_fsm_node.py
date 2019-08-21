#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: segment_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 11/03/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# segment with two legs
'''

import sys
import argparse
import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Float32, String


class Step(State):
    def __init__(self, right_leg_pub_topic, left_leg_pub_topic,  # TODO: Consolidate these topics (e.g. pub_leg_cmds, sub_leg_states)
                 right_leg_trigger_msg, left_leg_trigger_msg,
                 ros_rate, state_name):
        State.__init__(self, outcomes=['success'])

        # ROS stuff
        self.right_leg_pub = rospy.Publisher(right_leg_pub_topic, String, queue_size=1)
        self.left_leg_pub = rospy.Publisher(left_leg_pub_topic, String, queue_size=1)
        self.right_leg_trigger_msg = right_leg_trigger_msg
        self.left_leg_trigger_msg = left_leg_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name

    def execute(self, userdata):
        if self.right_leg_trigger_msg is not None:
            print("State: " + self.state_name + ", Sent trigger: " + self.right_leg_trigger_msg)
            self.right_leg_pub.publish(self.right_leg_trigger_msg)
        if self.left_leg_trigger_msg is not None:
            print("State: " + self.state_name + ", Sent trigger: " + self.left_leg_trigger_msg)
            self.left_leg_pub.publish(self.left_leg_trigger_msg)
        return('success')


class Sync(State):
    def __init__(self, right_leg_sub_topic, left_leg_sub_topic,  # TODO: Consolidate these topics (e.g. pub_leg_cmds, sub_leg_states)
                 right_leg_done_trigger_msg, left_leg_done_trigger_msg,
                 ros_rate, state_name):
        State.__init__(self, outcomes=['success'])

        # ROS stuff
        self.active_flag = False
        self.right_leg_sub = rospy.Subscriber(right_leg_sub_topic, String, self.right_leg_callback)
        self.left_leg_sub = rospy.Subscriber(left_leg_sub_topic, String, self.left_leg_callback)
        self.right_leg_done_trigger_msg = right_leg_done_trigger_msg
        self.left_leg_done_trigger_msg = left_leg_done_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name
        self.right_leg_trigger_msg = None
        self.left_leg_trigger_msg = None

    def execute(self, userdata):
        self.active_flag = True
        if self.right_leg_done_trigger_msg is not None:
            right_leg_done = False
        else:
            right_leg_done = True
        if self.left_leg_done_trigger_msg is not None:
            left_leg_done = False
        else:
            left_leg_done = True

        while not rospy.is_shutdown():
            if self.right_leg_trigger_msg is not None:
                if (self.right_leg_trigger_msg == self.right_leg_done_trigger_msg):
                    right_leg_done = True
                else:
                    print("Unrecognized trigger_msg: " + self.right_leg_trigger_msg)
                self.right_leg_trigger_msg = None
            if self.left_leg_trigger_msg is not None:
                if (self.left_leg_trigger_msg == self.left_leg_done_trigger_msg):
                    left_leg_done = True
                else:
                    print("Unrecognized trigger_msg: " + self.left_leg_trigger_msg)
                self.left_leg_trigger_msg = None
            if right_leg_done and left_leg_done:
                self.clean_up()
                return 'success'
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def right_leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ', Received trigger: ' + msg.data)
            self.right_leg_trigger_msg = msg.data

    def left_leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ', Received trigger: ' + msg.data)
            self.left_leg_trigger_msg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.right_leg_trigger_msg = None
        self.left_leg_trigger_msg = None


# Like Sync but specifically for triggers from higher-level state machines...
# I'm experimenting with different state machine architectures / patterns...
# One super state that does everything vs. multiple specialized states...
class TalkWithMaster(State):
    def __init__(self, trig_pub_topic, send_ping_msg,
                       trig_sub_topic, success1_trigger_msg, success2_trigger_msg,
                       ros_rate, state_name):
        State.__init__(self, outcomes=['success1', 'success2'])

        # ROS stuff
        self.active_flag = False
        self.trig_pub = rospy.Publisher(trig_pub_topic, String, queue_size=1)
        self.send_ping_msg = send_ping_msg
        self.trig_sub = rospy.Subscriber(trig_sub_topic, String, self.trig_callback)
        self.success1_trigger_msg = success1_trigger_msg
        self.success2_trigger_msg = success2_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.state_name = state_name
        self.received_trigger_msg = None

    def execute(self, userdata):
        self.active_flag = True
        if self.send_ping_msg is not None:
            print("State: " + self.state_name + ', Sending trigger: ' + self.send_ping_msg)
            self.trig_pub.publish(self.send_ping_msg)
        while not rospy.is_shutdown():
            if self.received_trigger_msg is not None:
                if (self.received_trigger_msg == self.success1_trigger_msg):
                    self.clean_up()
                    return 'success1'
                elif (self.received_trigger_msg == self.success2_trigger_msg):
                    self.clean_up()
                    return 'success2'
                else:
                    print("Unrecognized trigger_msg: " + self.received_trigger_msg)
                self.received_trigger_msg = None  # TODO: Either adjust loop control flow or reset received_trigger_msg here!!!
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def trig_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ', Received trigger: ' + msg.data)
            self.received_trigger_msg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.received_trigger_msg = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-from_right_leg', type=str, required=True, help="type=str, required=True, Description='right leg ROS topic to subscribe to'")
    parser.add_argument('-from_left_leg', type=str, required=True, help="type=str, required=True, Description='left leg ROS topic to subscribe to'")
    parser.add_argument('-to_right_leg', type=str, required=True, help="type=str, required=True, Description='right leg ROS topic to publish to'")
    parser.add_argument('-to_left_leg', type=str, required=True, help="type=str, required=True, Description='left leg ROS topic to publish to'")
    parser.add_argument('-from_master', type=str, required=True, help="type=str, required=True, Description='master ROS topic to subscribe to'")
    parser.add_argument('-to_master', type=str, required=True, help="type=str, required=True, Description='master ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, required=True, Description='rate at which ROS node publishes - rate = rospy.Rate(ros_rate)'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('segment_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    right_leg_sub_topic = parser.from_right_leg
    left_leg_sub_topic = parser.from_left_leg
    right_leg_pub_topic = parser.to_right_leg
    left_leg_pub_topic = parser.to_left_leg
    trig_sub_topic = parser.from_master
    trig_pub_topic = parser.to_master
    ros_rate = parser.ros_rate

    # segment state instances - arguments can always be bumped up (or down) the hierarchy
    # Initialization
    seg_init = TalkWithMaster(trig_pub_topic, "Ready", trig_sub_topic, "StartLeft", "StartRight", ros_rate, 'seg_init')
    # Right start
    seg_right_start = Step(right_leg_pub_topic, left_leg_pub_topic, "StartPush", "StartLift", ros_rate, 'seg_right_start')
    seg_right_start_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_right_start_sync')
    # Left start
    seg_left_start = Step(right_leg_pub_topic, left_leg_pub_topic, "StartLift", "StartPush", ros_rate, 'seg_left_start')
    seg_left_start_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_left_start_sync')
    # Left cycle
    seg_left_step = Step(right_leg_pub_topic, left_leg_pub_topic, "Recover", "Plant", ros_rate, 'seg_left_step')
    seg_left_step_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_left_step_sync')
    seg_left_push = Step(right_leg_pub_topic, left_leg_pub_topic, None, "Push", ros_rate, 'seg_left_push')
    seg_left_push_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, None, "Done", ros_rate, 'seg_left_push_sync')
    # Right cycle
    seg_right_step = Step(right_leg_pub_topic, left_leg_pub_topic, "Plant", "Recover", ros_rate, 'seg_right_step')
    seg_right_step_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_right_step_sync')
    seg_right_push = Step(right_leg_pub_topic, left_leg_pub_topic, "Push", None, ros_rate, 'seg_right_push')
    seg_right_push_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", None, ros_rate, 'seg_right_push_sync')
    # Sync with higher-level state machine
    seg_left_sync_with_master = TalkWithMaster(trig_pub_topic, "Ready", trig_sub_topic, "StepRight", "StepRight", ros_rate, 'seg_left_sync_with_master')
    seg_right_sync_with_master = TalkWithMaster(trig_pub_topic, "Ready", trig_sub_topic, "StepLeft", "StepLeft", ros_rate, 'seg_right_sync_with_master')

    # Create a SMACH state machine
    segment_fsm_node = StateMachine(outcomes=['success'])

    # Open the SMACH state machine
    with segment_fsm_node:
        # Add states to the container
        # TODO: Maybe add conditional statements controlled by cmd line arg 'topfsm' - which can be True or False
        # Pub up: "Ready" --> wait --> Sub up: "StartLeft" or "StartRight" --> transition: SEG_LEFT_START or SEG_RIGHT_START
        StateMachine.add('SEG_INIT', seg_init, transitions={'success1':'SEG_LEFT_START', 'success2':'SEG_RIGHT_START'})

        # Pub down: "StartPush" and "StartLift" --> transition: SEG_RIGHT_START_SYNC
        StateMachine.add('SEG_RIGHT_START', seg_right_start, transitions={'success':'SEG_RIGHT_START_SYNC'})
        # wait --> Sub down: "Done" and "Done" --> SEG_LEFT_STEP
        StateMachine.add('SEG_RIGHT_START_SYNC', seg_right_start_sync, transitions={'success':'SEG_LEFT_STEP'})

        StateMachine.add('SEG_LEFT_START', seg_left_start, transitions={'success':'SEG_LEFT_START_SYNC'})
        StateMachine.add('SEG_LEFT_START_SYNC', seg_left_start_sync, transitions={'success':'SEG_RIGHT_STEP'})

        StateMachine.add('SEG_LEFT_STEP', seg_left_step, transitions={'success':'SEG_LEFT_STEP_SYNC'})
        StateMachine.add('SEG_LEFT_STEP_SYNC', seg_left_step_sync, transitions={'success':'SEG_LEFT_PUSH'})
        StateMachine.add('SEG_LEFT_PUSH', seg_left_push, transitions={'success':'SEG_LEFT_PUSH_SYNC'})
        StateMachine.add('SEG_LEFT_PUSH_SYNC', seg_left_push_sync, transitions={'success':'SEG_LEFT_SYNC_WITH_MASTER'})

        StateMachine.add('SEG_RIGHT_STEP', seg_right_step, transitions={'success':'SEG_RIGHT_STEP_SYNC'})
        StateMachine.add('SEG_RIGHT_STEP_SYNC', seg_right_step_sync, transitions={'success':'SEG_RIGHT_PUSH'})
        StateMachine.add('SEG_RIGHT_PUSH', seg_right_push, transitions={'success':'SEG_RIGHT_PUSH_SYNC'})
        StateMachine.add('SEG_RIGHT_PUSH_SYNC', seg_right_push_sync, transitions={'success':'SEG_RIGHT_SYNC_WITH_MASTER'})

        StateMachine.add('SEG_LEFT_SYNC_WITH_MASTER', seg_left_sync_with_master, transitions={'success1':'SEG_RIGHT_STEP', 'success2':'SEG_RIGHT_STEP'})
        StateMachine.add('SEG_RIGHT_SYNC_WITH_MASTER', seg_right_sync_with_master, transitions={'success1':'SEG_LEFT_STEP', 'success2':'SEG_LEFT_STEP'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('segment_fsm_node' + str(rospy.get_name()), segment_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: segment_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: segment_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = segment_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sys.stop()
    print("\nExiting " + str(rospy.get_name()))

if __name__ == '__main__':
    main()
