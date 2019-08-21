#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: segment_symmetrical_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 10/10/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# segment with two legs
'''

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Float32, String

import time, math, sys

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

        while True:
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
        while True:
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

    def trig_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ', Received trigger: ' + msg.data)
            self.received_trigger_msg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.received_trigger_msg = None

def main():
    # ROS stuff
    rospy.init_node('segment_fsm_node', anonymous=True)

    right_leg_sub_topic = None
    left_leg_sub_topic = None
    right_leg_pub_topic = None
    left_leg_pub_topic = None
    trig_sub_topic = None
    trig_pub_topic = None
    ros_rate = None

    if len(sys.argv) < 8:
        print("usage: segment_fsm_node.py right_leg_sub_topic left_leg_sub_topic \
               right_leg_pub_topic left_leg_pub_topic trig_sub_topic trig_pub_topic ros_rate")
    else:
        right_leg_sub_topic = sys.argv[1]
        left_leg_sub_topic = sys.argv[2]
        right_leg_pub_topic = sys.argv[3]
        left_leg_pub_topic = sys.argv[4]
        trig_sub_topic = sys.argv[5]
        trig_pub_topic = sys.argv[6]
        ros_rate = float(sys.argv[7])

    # segment state instances - arguments can always be bumped up (or down) the hierarchy
    # Initialization
    seg_init = TalkWithMaster(trig_pub_topic, "Ready", trig_sub_topic, "StartPush", "StartLift", ros_rate, 'seg_init')
    # Push start
    seg_push_start = Step(right_leg_pub_topic, left_leg_pub_topic, "StartPush", "StartPush", ros_rate, 'seg_push_start')
    seg_push_start_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_push_start_sync')
    # Lift start
    seg_lift_start = Step(right_leg_pub_topic, left_leg_pub_topic, "StartLift", "StartLift", ros_rate, 'seg_lift_start')
    seg_lift_start_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_lift_start_sync')
    # Recover
    seg_recover = Step(right_leg_pub_topic, left_leg_pub_topic, "Recover", "Recover", ros_rate, 'seg_recover')
    seg_recover_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_recover_sync')
    # Push
    seg_push = Step(right_leg_pub_topic, left_leg_pub_topic, "Push", "Push", ros_rate, 'seg_push')
    seg_push_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_push_sync')
    # Plant
    seg_plant = Step(right_leg_pub_topic, left_leg_pub_topic, "Plant", "Plant", ros_rate, 'seg_plant')
    seg_plant_sync = Sync(right_leg_sub_topic, left_leg_sub_topic, "Done", "Done", ros_rate, 'seg_plant_sync')

    # Sync with higher-level state machine
    seg_sync_with_master = TalkWithMaster(trig_pub_topic, "Ready", trig_sub_topic, "Plant", "Recover", ros_rate, 'seg_sync_with_master')

    # Create a SMACH state machine
    segment_fsm_node = StateMachine(outcomes=['success'])

    # Open the SMACH state machine
    with segment_fsm_node:
        # Add states to the container
        # TODO: Maybe add conditional statements controlled by cmd line arg 'topfsm' - which can be True or False
        # Pub up: "Ready" --> wait --> Sub up: "StartPush" or "StartLift" --> transition: SEG_PUSH_START or SEG_LIFT_START
        StateMachine.add('SEG_INIT', seg_init, transitions={'success1':'SEG_PUSH_START', 'success2':'SEG_LIFT_START'})
        # Pub down: "StartPush" and "StartPush" --> transition: SEG_PUSH_START_SYNC
        StateMachine.add('SEG_PUSH_START', seg_push_start, transitions={'success':'SEG_PUSH_START_SYNC'})
        # wait --> Sub down: "Done" and "Done" --> SEG_RECOVER
        StateMachine.add('SEG_PUSH_START_SYNC', seg_push_start_sync, transitions={'success':'SEG_RECOVER'})

        StateMachine.add('SEG_LIFT_START', seg_lift_start, transitions={'success':'SEG_LIFT_START_SYNC'})
        StateMachine.add('SEG_LIFT_START_SYNC', seg_lift_start_sync, transitions={'success':'SEG_PLANT'})

        StateMachine.add('SEG_RECOVER', seg_recover, transitions={'success':'SEG_RECOVER_SYNC'})
        StateMachine.add('SEG_RECOVER_SYNC', seg_recover_sync, transitions={'success':'SEG_SYNC_WITH_MASTER'})

        StateMachine.add('SEG_PLANT', seg_plant, transitions={'success':'SEG_PLANT_SYNC'})
        StateMachine.add('SEG_PLANT_SYNC', seg_plant_sync, transitions={'success':'SEG_PUSH'})
        StateMachine.add('SEG_PUSH', seg_push, transitions={'success':'SEG_PUSH_SYNC'})
        StateMachine.add('SEG_PUSH_SYNC', seg_push_sync, transitions={'success':'SEG_SYNC_WITH_MASTER'})

        StateMachine.add('SEG_SYNC_WITH_MASTER', seg_sync_with_master, transitions={'success1':'SEG_PLANT', 'success2':'SEG_RECOVER'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('segment_symmetrical_fsm_node' + str(rospy.get_name()), segment_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: segment_symmetrical_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: segment_symmetrical_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = segment_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
