#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: two_dof_leg_small_step_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 10/10/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# 2-dof leg with a z-axis hip joint and an x-axis knee joint
'''

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Float32, String

import time, math, sys

class MoveLeg(State):
    def __init__(self, knee_target_pos, hip_target_pos, side,
                knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic,  # TODO: Consolidate these topics (e.g. pub_motor_cmd, sub_motor_state)
                trig_pub_topic, ros_rate, success_trigger_msg, state_name):
        State.__init__(self, outcomes=['success', 'failure'],
                             input_keys=['knee_current_pos', 'hip_current_pos'],
                             output_keys=['knee_current_pos', 'hip_current_pos', 'knee_target_pos', 'hip_target_pos'])

        if (side == 'right'):
            self.knee_target_pos = knee_target_pos
            self.hip_target_pos = hip_target_pos
        elif (side == 'left'):
            self.knee_target_pos = -knee_target_pos
            self.hip_target_pos = -hip_target_pos
        else:
            raise ValueError("Invalid argument: " + str(side) + " ! Constructor requires 'left' or 'right' as arguments for the 'side' parameter")

        # state machine stuff
        self.knee_current_pos = None  # These get initialized in execute()
        self.hip_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.knee_sub = rospy.Subscriber(knee_sub_topic, Float32, self.knee_callback)   # Subscribe to the knee joint's angle topic
        self.hip_sub = rospy.Subscriber(hip_sub_topic, Float32, self.hip_callback)      # Subscribe to the hip joint's angle topic
        self.knee_pub = rospy.Publisher(knee_pub_topic, Float32, queue_size=1)  # Publish commands to the knee joint's topic
        self.hip_pub = rospy.Publisher(hip_pub_topic, Float32, queue_size=1)    # Publish commands to the hip joint's topic
        self.trig_pub = rospy.Publisher(trig_pub_topic, String, queue_size=100) # Publish commands to the trigger topic - could define custom msg type
        self.rate = rospy.Rate(ros_rate)
        self.success_trigger_msg = success_trigger_msg
        self.state_name = state_name
        self.knee_sub = None  # These get initialized in execute()
        self.hip_sub = None

    def execute(self, userdata):
        self.active_flag = True
        self.knee_current_pos = userdata.knee_current_pos   # values from previous state
        self.hip_current_pos = userdata.hip_current_pos

        timeout_counter = time.time()
        timeout_time = 10.0

        settle_tolerance = 0.08  # < 5 degrees
        settle_time = 0.1

        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False

        while True:
            if ((time.time() - timeout_counter) > timeout_time):
                self.update_userdata(userdata)
                self.clean_up()
                return 'failure'
            else:
                # Check knee success condition
                if (abs(self.knee_current_pos - self.knee_target_pos) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = time.time()
                    elif ((time.time() - knee_settle_start) > settle_time):
                        knee_done = True
                # Check hip success condition
                if (abs(self.hip_current_pos - self.hip_target_pos) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = time.time()
                    elif ((time.time() - hip_settle_start) > settle_time):
                        hip_done = True
                if (knee_done and hip_done):
                    print("State: " + self.state_name + ', Sent trigger: ' + self.success_trigger_msg)
                    self.trig_pub.publish(self.success_trigger_msg)  # Tell the trigger topic you're done
                    self.clean_up()
                    self.update_userdata(userdata)
                    return 'success'  # You're done!
                else:
                    # TODO: Add a position/effort ramp function
                    self.knee_pub.publish(self.knee_target_pos)
                    self.hip_pub.publish(self.hip_target_pos)
            self.rate.sleep()

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def update_userdata(self, userdata):
        userdata.knee_current_pos = self.knee_current_pos
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_target_pos = self.knee_target_pos
        userdata.hip_target_pos = self.hip_target_pos

    def clean_up(self):
        self.active_flag = False


class WaitLeg(State):
    def __init__(self, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic,
                trig_sub_topic, ros_rate, success1_trigger_msg, success2_trigger_msg, state_name):
        State.__init__(self, outcomes=['success1', 'success2'],
                             input_keys=['knee_current_pos', 'hip_current_pos', 'knee_target_pos', 'hip_target_pos'],
                             output_keys=['knee_current_pos', 'hip_current_pos'])

        # ROS stuff
        self.active_flag = False
        self.knee_sub = rospy.Subscriber(knee_sub_topic, Float32, self.knee_callback)   # Subscribe to the knee joint's angle topic
        self.hip_sub = rospy.Subscriber(hip_sub_topic, Float32, self.hip_callback)      # Subscribe to the hip joint's angle topic
        self.trig_sub = rospy.Subscriber(trig_sub_topic, String, self.trig_callback)    # Subscribe to the trigger topic
        self.knee_pub = rospy.Publisher(knee_pub_topic, Float32, queue_size=1)  # Publish commands to the knee joint's topic
        self.hip_pub = rospy.Publisher(hip_pub_topic, Float32, queue_size=1)    # Publish commands to the hip joint's topic
        self.trig_sub_topic = trig_sub_topic
        self.rate = rospy.Rate(ros_rate)
        self.received_trigger_msg = None
        self.success1_trigger_msg = success1_trigger_msg
        self.success2_trigger_msg = success2_trigger_msg
        self.state_name = state_name
        self.knee_sub = None
        self.hip_sub = None
        self.trig_sub = None

        # state machine stuff
        self.knee_current_pos = None  # These get initialized in execute()
        self.hip_current_pos = None
        self.knee_target_pos = None
        self.hip_target_pos = None

    def execute(self, userdata):
        self.active_flag = True
        self.knee_current_pos = userdata.knee_current_pos   # values from previous state
        self.hip_current_pos = userdata.hip_current_pos
        self.knee_target_pos = userdata.knee_target_pos
        self.hip_target_pos = userdata.hip_target_pos

        # Do we want a more robust communication system (handshakes, acks)?
        while True:
            if self.received_trigger_msg is not None:
                if (self.received_trigger_msg == self.success1_trigger_msg):
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'success1'
                elif (self.received_trigger_msg == self.success2_trigger_msg):
                    self.update_userdata(userdata)
                    self.clean_up()
                    return 'success2'
                else:
                    print("Unrecognized trigger_msg: " + self.received_trigger_msg)
                self.received_trigger_msg = None
            else:
                # TODO: Add a position/effort ramp function
                self.knee_pub.publish(self.knee_target_pos)
                self.hip_pub.publish(self.hip_target_pos)
            self.rate.sleep()

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def trig_callback(self, msg):
        if self.active_flag:
            print("State: " + self.state_name + ', Received trigger: ' + msg.data)
            self.received_trigger_msg = msg.data

    def update_userdata(self, userdata):
        userdata.knee_current_pos = self.knee_current_pos
        userdata.hip_current_pos = self.hip_current_pos

    # Prevents strange behavior from inactive states
    def clean_up(self):
        self.active_flag = False
        self.received_trigger_msg = None


def main():
    # ROS stuff
    rospy.init_node('two_dof_leg_fsm_node', anonymous=True)  # 'anonymous' prevents collisions/shutdown so we can launch multiple two_dof_leg_fsm_node.py nodes

    side = None
    knee_pub_topic = None
    knee_sub_topic = None
    hip_pub_topic = None
    hip_sub_topic = None
    trig_sub_topic = None
    trig_pub_topic = None
    ros_rate = None

    if len(sys.argv) < 9:
        print("usage: two_dof_leg_fsm_node.py side knee_sub_topic hip_sub_topic knee_pub_topic hip_pub_topic trig_sub_topic trig_pub_topic ros_rate")
    else:
        side = sys.argv[1]
        knee_sub_topic = sys.argv[2]
        hip_sub_topic = sys.argv[3]
        knee_pub_topic = sys.argv[4]
        hip_pub_topic = sys.argv[5]
        trig_sub_topic = sys.argv[6]
        trig_pub_topic = sys.argv[7]
        ros_rate = float(sys.argv[8])

    # leg state instances - arguments can always be bumped up (or down) the hierarchy
    leg_wait_start = WaitLeg(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_sub_topic, ros_rate, 'StartLift', 'StartPush', 'leg_wait_start')
    leg_lift = MoveLeg(-1.39626, 0.0, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_pub_topic, ros_rate, 'Done', 'leg_lift')
    leg_wait_plant = WaitLeg(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_sub_topic, ros_rate, 'Plant', None, 'leg_wait_plant')
    leg_plant = MoveLeg(-0.34906585, 0.1309, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_pub_topic, ros_rate, 'Done', 'leg_plant')
    leg_wait_push = WaitLeg(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_sub_topic, ros_rate, 'Push', None, 'leg_wait_push')
    leg_push = MoveLeg(-0.34906585, -0.1309, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_pub_topic, ros_rate, 'Done', 'leg_push')
    leg_wait_recover = WaitLeg(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, trig_sub_topic, ros_rate, 'Recover', None, 'leg_wait_recover')

    # Create a SMACH state machine
    two_dof_leg_fsm_node = StateMachine(outcomes=['success'])
    two_dof_leg_fsm_node.userdata.knee_current_pos = 0.0  # initial value
    two_dof_leg_fsm_node.userdata.hip_current_pos = 0.0  # initial value
    if (side == 'right'):
        two_dof_leg_fsm_node.userdata.knee_target_pos = -0.34906585  # initial value
    elif (side == 'left'):
        two_dof_leg_fsm_node.userdata.knee_target_pos = 0.34906585  # initial value
    else:
        raise ValueError("Invalid argument: " + str(side) + " ! Constructor requires 'left' or 'right' as arguments for the 'side' parameter")
    two_dof_leg_fsm_node.userdata.hip_target_pos = 0.0  # initial value

    # Open the SMACH state machine
    with two_dof_leg_fsm_node:
        # Add states to the container
        StateMachine.add('LEG_WAIT_START', leg_wait_start, transitions={'success1':'LEG_LIFT', 'success2':'LEG_PUSH'},
                                                           remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                                      'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})
        StateMachine.add('LEG_LIFT', leg_lift, transitions={'failure':'LEG_LIFT','success':'LEG_WAIT_PLANT'},
                                               remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('LEG_WAIT_PLANT', leg_wait_plant, transitions={'success1':'LEG_PLANT', 'success2':'LEG_PLANT'},
                                                           remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                                      'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})
        StateMachine.add('LEG_PLANT', leg_plant, transitions={'failure':'LEG_PLANT','success':'LEG_WAIT_PUSH'},
                                                 remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('LEG_WAIT_PUSH', leg_wait_push, transitions={'success1':'LEG_PUSH', 'success2':'LEG_PUSH'},
                                                         remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                                    'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})
        StateMachine.add('LEG_PUSH', leg_push, transitions={'failure':'LEG_PUSH','success':'LEG_WAIT_RECOVER'},
                                               remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('LEG_WAIT_RECOVER', leg_wait_recover, transitions={'success1':'LEG_LIFT', 'success2':'LEG_LIFT'},
                                                               remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                                          'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    #sis = smach_ros.IntrospectionServer('two_dof_leg_fsm_node' + str(rospy.get_name()), two_dof_leg_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    #sis.start()

    # Give Gazebo some time to start up
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: two_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    #print("Input received. Executing - pkg: two_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    outcome = two_dof_leg_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
