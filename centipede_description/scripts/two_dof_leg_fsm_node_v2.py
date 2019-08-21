#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: two_dof_leg_fsm_node_v2.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 11/28/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# 2-dof leg with a z-axis hip joint and an x-axis knee joint
'''

import sys
import argparse
import rospy
from std_msgs.msg import Float32, String
from smach import State, StateMachine
import smach_ros


class MoveLeg(State):
    """SMACH state: Sends a joint state to the hip & knee joint controller nodes

    Attributes:
        knee_target_pos     (float): knee joint target angle (rad)
        hip_target_pos      (float): hip joint target angle (rad)
        side                (str): left or right
        knee_sub_topic      (str): subscribe to knee joint angle
        hip_sub_topic       (str): subscribe to hip joint angle
        knee_pub_topic      (str): publish to knee joint controller
        hip_pub_topic       (str): publish to hip joint controller
        pub_to_master_topic (str): publish to master
        success_trigger_msg (str): message published on pub_to_master_topic when 'success'
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, knee_target_pos, hip_target_pos, side,
                knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic,
                pub_to_master_topic, success_trigger_msg, ros_rate, instance_name):
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
        self.hip_current_pos = None  # These get initialized in execute()
        self.knee_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.hip_sub = rospy.Subscriber(hip_sub_topic, Float32, self.hip_callback)      # Subscribe to the hip joint's angle topic
        self.hip_pub = rospy.Publisher(hip_pub_topic, Float32, queue_size=1)            # Publish commands to the hip joint's topic
        self.knee_sub = rospy.Subscriber(knee_sub_topic, Float32, self.knee_callback)   # Subscribe to the knee joint's angle topic
        self.knee_pub = rospy.Publisher(knee_pub_topic, Float32, queue_size=1)          # Publish commands to the knee joint's topic
        self.master_pub = rospy.Publisher(pub_to_master_topic, String, queue_size=100)  # Publish commands to the trigger topic - could define custom msg type
        self.success_trigger_msg = success_trigger_msg
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name

    def execute(self, userdata):
        self.active_flag = True
        self.knee_current_pos = userdata.knee_current_pos   # values from previous state
        self.hip_current_pos = userdata.hip_current_pos

        timeout_counter = rospy.Time.now().to_sec()
        timeout_time = 5.0

        settle_tolerance = 0.08  # < 5 degrees
        settle_time = 0.5

        knee_settle_start = None
        knee_settle_countdown = False
        knee_done = False
        hip_settle_start = None
        hip_settle_countdown = False
        hip_done = False

        while not rospy.is_shutdown():
            if ((rospy.Time.now().to_sec() - timeout_counter) > timeout_time):
                self.update_userdata(userdata)
                self.clean_up()
                return 'failure'
            else:
                # Check knee success condition
                if (abs(self.knee_current_pos - self.knee_target_pos) < settle_tolerance):
                    if not knee_settle_countdown:
                        knee_settle_countdown = True
                        knee_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - knee_settle_start) > settle_time):
                        knee_done = True
                # Check hip success condition
                if (abs(self.hip_current_pos - self.hip_target_pos) < settle_tolerance):
                    if not hip_settle_countdown:
                        hip_settle_countdown = True
                        hip_settle_start = rospy.Time.now().to_sec()
                    elif ((rospy.Time.now().to_sec() - hip_settle_start) > settle_time):
                        hip_done = True
                if (knee_done and hip_done):
                    print("State: " + self.instance_name + ', Sent trigger: ' + self.success_trigger_msg + " to master.")
                    self.master_pub.publish(self.success_trigger_msg)  # Inform master of your success
                    self.clean_up()
                    self.update_userdata(userdata)
                    return 'success'  # You're done!
                else:
                    # TODO: Add a position/effort ramp function
                    self.hip_pub.publish(self.hip_target_pos)
                    self.knee_pub.publish(self.knee_target_pos)
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos
        userdata.hip_target_pos = self.hip_target_pos
        userdata.knee_target_pos = self.knee_target_pos

    def clean_up(self):
        self.active_flag = False


class WaitForMasterCmd(State):
    """SMACH state: Waits for a trigger message from a master FSM

    Attributes:
        knee_sub_topic      (str): subscribe to knee joint angle
        hip_sub_topic       (str): subscribe to hip joint angle
        knee_pub_topic      (str): publish to knee joint controller
        hip_pub_topic       (str): publish to hip joint controller
        sub_to_master_topic (str): subscribe to master
        master_trigger_msgs (list of str): recognized commands from master
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic,
                sub_to_master_topic, master_trigger_msgs, ros_rate, instance_name):
        State.__init__(self, outcomes=master_trigger_msgs+['failure'],
                             input_keys=['knee_current_pos', 'hip_current_pos', 'knee_target_pos', 'hip_target_pos'],
                             output_keys=['knee_current_pos', 'hip_current_pos'])
        # state machine stuff
        self.hip_target_pos = None  # These get initialized in execute()
        self.knee_target_pos = None
        self.hip_current_pos = None
        self.knee_current_pos = None

        # ROS stuff
        self.active_flag = False
        self.hip_sub = rospy.Subscriber(hip_sub_topic, Float32, self.hip_callback)              # Subscribe to the hip joint's angle topic
        self.hip_pub = rospy.Publisher(hip_pub_topic, Float32, queue_size=1)                    # Publish commands to the hip joint's topic
        self.knee_sub = rospy.Subscriber(knee_sub_topic, Float32, self.knee_callback)           # Subscribe to the knee joint's angle topic
        self.knee_pub = rospy.Publisher(knee_pub_topic, Float32, queue_size=1)                  # Publish commands to the knee joint's topic
        self.master_sub = rospy.Subscriber(sub_to_master_topic, String, self.master_callback)   # Subscribe to the trigger topic
        self.master_trigger_msgs = master_trigger_msgs
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name
        self.master_trigger_msgs_set = frozenset(self.master_trigger_msgs)
        self.msg_from_master = None


    def execute(self, userdata):
        self.active_flag = True
        self.hip_target_pos = userdata.hip_target_pos  # values from previous state
        self.knee_target_pos = userdata.knee_target_pos
        self.hip_current_pos = userdata.hip_current_pos
        self.knee_current_pos = userdata.knee_current_pos

        while not rospy.is_shutdown():
            if self.msg_from_master is not None:
                # process msg_from_master
                if self.msg_from_master in self.master_trigger_msgs_set:
                    trigger_msg = self.msg_from_master
                    self.update_userdata(userdata)
                    self.clean_up()
                    return trigger_msg
                else:
                    print("Unrecognized msg_from_master: " + self.msg_from_master)
                    self.msg_from_master = None
            else:
                # TODO: Add a position/effort ramp function
                self.hip_pub.publish(self.hip_target_pos)
                self.knee_pub.publish(self.knee_target_pos)
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def knee_callback(self, msg):
        if self.active_flag:
            self.knee_current_pos = msg.data

    def hip_callback(self, msg):
        if self.active_flag:
            self.hip_current_pos = msg.data

    def master_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ', Received trigger: ' + msg.data + " from master")
            self.msg_from_master = msg.data

    def update_userdata(self, userdata):
        userdata.hip_current_pos = self.hip_current_pos
        userdata.knee_current_pos = self.knee_current_pos

    # Prevents strange behavior from inactive states
    def clean_up(self):
        self.active_flag = False
        self.msg_from_master = None


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-side', type=str, choices=['right', 'left'], required=True, help="type=str, Description='side the leg is mounted on'")
    parser.add_argument('-from_hip', type=str, required=True, help="type=str, Description='hip joint ROS topic to subscribe to'")
    parser.add_argument('-to_hip', type=str, required=True, help="type=str, Description='hip joint ROS topic to publish to'")
    parser.add_argument('-from_knee', type=str, required=True, help="type=str, Description='knee joint ROS topic to subscribe to'")
    parser.add_argument('-to_knee', type=str, required=True, help="type=str, Description='knee joint ROS topic to publish to'")
    parser.add_argument('-from_segment', type=str, required=True, help="type=str, Description='segment ROS topic to subscribe to'")
    parser.add_argument('-to_segment', type=str, required=True, help="type=str, Description='segment ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, default=100, Description='rate at which ROS node publishes'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('two_dof_leg_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    side = parser.side
    knee_sub_topic = parser.from_knee
    hip_sub_topic = parser.from_hip
    knee_pub_topic = parser.to_knee
    hip_pub_topic = parser.to_hip
    from_master = parser.from_segment
    to_master = parser.to_segment
    ros_rate = parser.ros_rate

    # leg state instances - arguments can always be bumped up (or down) the hierarchy
    wait_start = WaitForMasterCmd(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, from_master,
                                  ['lift', 'backward', 'forward'],
                                  ros_rate, 'wait_start')
    lift = MoveLeg(-1.57079633, 0.0, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done', ros_rate, 'lift')
    backward = MoveLeg(-0.34906585, -0.261799, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done', ros_rate, 'backward')
    center = MoveLeg(-0.34906585, 0.0, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done', ros_rate, 'center')
    forward = MoveLeg(-0.34906585, 0.261799, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done', ros_rate, 'forward')
    wait_orders = WaitForMasterCmd(knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, from_master,
                                   ['lift', 'backward', 'center', 'forward',
                                    'backward_shallow', 'center_shallow', 'forward_shallow'],
                                   ros_rate, 'wait_orders')
    backward_shallow = MoveLeg(-0.785398, -0.261799, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done',  ros_rate, 'backward_shallow')
    center_shallow = MoveLeg(-0.785398, 0.0, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done',  ros_rate, 'center_shallow')
    forward_shallow = MoveLeg(-0.785398, 0.261799, side, knee_sub_topic, hip_sub_topic, knee_pub_topic, hip_pub_topic, to_master, 'done', ros_rate, 'forward_shallow')

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
        StateMachine.add('WAIT_START', wait_start, transitions={'failure':'WAIT_START',
                                                                'lift':'LIFT',
                                                                'backward':'BACKWARD', 'forward':'FORWARD'},
                                                   remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                              'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})
        StateMachine.add('LIFT', lift, transitions={'failure':'LIFT','success':'WAIT_ORDERS'},
                                       remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('BACKWARD', backward, transitions={'failure':'BACKWARD','success':'WAIT_ORDERS'},
                                               remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('CENTER', backward, transitions={'failure':'CENTER','success':'WAIT_ORDERS'},
                                             remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('FORWARD', forward, transitions={'failure':'FORWARD','success':'WAIT_ORDERS'},
                                             remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('WAIT_ORDERS', wait_orders, transitions={'failure':'WAIT_ORDERS',
                                                                  'lift':'LIFT',
                                                                  'backward':'BACKWARD', 'center':'CENTER', 'forward':'FORWARD',
                                                                  'backward_shallow':'BACKWARD_SHALLOW', 'center_shallow':'CENTER_SHALLOW', 'forward_shallow':'FORWARD_SHALLOW'},
                                                     remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos',
                                                                'knee_target_pos':'knee_target_pos','hip_target_pos':'hip_target_pos'})
        StateMachine.add('BACKWARD_SHALLOW', backward_shallow, transitions={'failure':'BACKWARD_SHALLOW','success':'WAIT_ORDERS'},
                                                               remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('CENTER_SHALLOW', center_shallow, transitions={'failure':'CENTER_SHALLOW','success':'WAIT_ORDERS'},
                                                           remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})
        StateMachine.add('FORWARD_SHALLOW', forward_shallow, transitions={'failure':'FORWARD_SHALLOW','success':'WAIT_ORDERS'},
                                                             remapping={'knee_current_pos':'knee_current_pos','hip_current_pos':'hip_current_pos'})

    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('two_dof_leg_fsm_node' + str(rospy.get_name()), two_dof_leg_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing - pkg: two_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing - pkg: two_dof_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    outcome = two_dof_leg_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
