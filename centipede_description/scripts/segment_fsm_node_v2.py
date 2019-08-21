#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: segment_fsm_node_v2.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/10/2017
# Edit Date: 11/28/2017
#
# Description:
# Finite state machine controlling the position and movements of a
# segment with two legs
'''

import sys
import argparse
import rospy
from std_msgs.msg import Float32, String
from smach import State, StateMachine
import smach_ros


class SendLegCmds(State):
    """SMACH state: Send cmds to one or both slave FSMs

    Attributes:
        right_leg_pub_topic (str): publish to right leg topic
        left_leg_pub_topic  (str): publish to left leg topic
        right_leg_cmd       (str): message published on right_leg_pub_topic
        left_leg_cmd        (str): message published on left_leg_pub_topic
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, right_leg_pub_topic, left_leg_pub_topic,
                 right_leg_cmd, left_leg_cmd,
                 ros_rate, instance_name):
        State.__init__(self, outcomes=['success'])

        # ROS stuff
        self.right_leg_pub = rospy.Publisher(right_leg_pub_topic, String, queue_size=1)
        self.left_leg_pub = rospy.Publisher(left_leg_pub_topic, String, queue_size=1)
        self.right_leg_cmd = right_leg_cmd
        self.left_leg_cmd = left_leg_cmd
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name

    def execute(self, userdata):
        if self.right_leg_cmd is not None:
            print("State: " + self.instance_name + ", Sent trigger: " + self.right_leg_cmd + " to right slave...")
            self.right_leg_pub.publish(self.right_leg_cmd)
        if self.left_leg_cmd is not None:
            print("State: " + self.instance_name + ", Sent trigger: " + self.left_leg_cmd + " to left slave...")
            self.left_leg_pub.publish(self.left_leg_cmd)
        return('success')


class SyncLegs(State):
    """SMACH state: Waits for trigger messages from both slave FSMs

    Attributes:
        right_leg_sub_topic (str): subscribe to right leg topic
        left_leg_sub_topic  (str): subscribe to left leg topic
        right_leg_done_msg  (str): recognized done msgs from right leg
        left_leg_done_msg   (str): recognized done msgs from left leg
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, right_leg_sub_topic, left_leg_sub_topic,
                 right_leg_done_msg, left_leg_done_msg,
                 ros_rate, instance_name):
        State.__init__(self, outcomes=['failure', 'success'])

        # ROS stuff
        self.active_flag = False
        self.right_leg_sub = rospy.Subscriber(right_leg_sub_topic, String, self.right_leg_callback)
        self.left_leg_sub = rospy.Subscriber(left_leg_sub_topic, String, self.left_leg_callback)
        self.right_leg_done_msg = right_leg_done_msg
        self.left_leg_done_msg = left_leg_done_msg
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name
        self.msg_from_right_leg = None
        self.msg_from_left_leg = None

    def execute(self, userdata):
        self.active_flag = True
        right_leg_done = False
        left_leg_done = False
        if self.right_leg_done_msg is None:
            right_leg_done = True
        if self.left_leg_done_msg is None:
            left_leg_done = True

        while not rospy.is_shutdown():
            if self.msg_from_right_leg is not None:
                if (self.msg_from_right_leg == self.right_leg_done_msg):
                    right_leg_done = True
                else:
                    print("Unrecognized msg_from_right_leg: " + self.msg_from_right_leg)
                self.msg_from_right_leg = None
            if self.msg_from_left_leg is not None:
                if (self.msg_from_left_leg == self.left_leg_done_msg):
                    left_leg_done = True
                else:
                    print("Unrecognized msg_from_left_leg: " + self.msg_from_left_leg)
                self.msg_from_left_leg = None
            if right_leg_done and left_leg_done:
                self.clean_up()
                return 'success'
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def right_leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ', Received trigger: ' + msg.data + ' from right slave')
            self.msg_from_right_leg = msg.data

    def left_leg_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ', Received trigger: ' + msg.data + ' from left slave')
            self.msg_from_left_leg = msg.data

    def clean_up(self):
        self.active_flag = False
        self.msg_from_right_leg = None
        self.msg_from_left_leg = None

# Like Sync but specifically for triggers from higher-level state machines...
# I'm experimenting with different state machine architectures / patterns...
# One super state that does everything vs. multiple specialized states...
class WaitForMasterCmd(State):
    """SMACH state: Waits for a trigger message from master FSM

    Attributes:
        from_master         (str): subscribe to master
        to_master           (str): publish to master
        success_trigger_msg (str): message published on to_master topic
        master_trigger_msgs (list of str): recognized commands from master
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, from_master, to_master, success_trigger_msg,
                master_trigger_msgs, ros_rate, instance_name):
        State.__init__(self, outcomes=master_trigger_msgs+['failure'])
        # ROS stuff
        self.active_flag = False
        self.master_sub = rospy.Subscriber(from_master, String, self.master_callback)
        self.master_pub = rospy.Publisher(to_master, String, queue_size=1)
        self.success_trigger_msg = success_trigger_msg
        self.master_trigger_msgs = master_trigger_msgs
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name
        self.master_trigger_msgs_set = frozenset(self.master_trigger_msgs)
        self.msg_from_master = None

    def execute(self, userdata):
        self.active_flag = True
        if self.success_trigger_msg is not None:
            print("State: " + self.instance_name + ', Sending trigger: ' + self.success_trigger_msg)
            self.master_pub.publish(self.success_trigger_msg)
        while not rospy.is_shutdown():
            if self.msg_from_master is not None:
                # process msg_from_master
                if self.msg_from_master in self.master_trigger_msgs_set:
                    trigger_msg = self.msg_from_master
                    self.clean_up()
                    return trigger_msg
                else:
                    print("Unrecognized msg_from_master: " + self.msg)
                    self.msg_from_master = None
            self.rate.sleep()
        return 'failure'  # if rospy.is_shutdown()

    def master_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ', Received trigger: ' + msg.data + " from master")
            self.msg_from_master = msg.data

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
    parser.add_argument('-from_right_leg', type=str, required=True, help="type=str, Description='right leg ROS topic to subscribe to'")
    parser.add_argument('-from_left_leg', type=str, required=True, help="type=str, Description='left leg ROS topic to subscribe to'")
    parser.add_argument('-to_right_leg', type=str, required=True, help="type=str, Description='right leg ROS topic to publish to'")
    parser.add_argument('-to_left_leg', type=str, required=True, help="type=str, Description='left leg ROS topic to publish to'")
    parser.add_argument('-from_master', type=str, required=True, help="type=str, Description='master ROS topic to subscribe to'")
    parser.add_argument('-to_master', type=str, required=True, help="type=str, Description='master ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, Description='rate at which ROS node publishes'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('segment_fsm_node', anonymous=True)

    # Parse cmd line arguments
    parser = parse_args(sys.argv[1:])
    from_right_leg = parser.from_right_leg
    from_left_leg = parser.from_left_leg
    to_right_leg = parser.to_right_leg
    to_left_leg = parser.to_left_leg
    from_master = parser.from_master
    to_master = parser.to_master
    ros_rate = parser.ros_rate

    #---------------------------------------------------------------------------
    # Create SMACH state machine
    segment_fsm_node = StateMachine(outcomes=['success'])

    # segment state instances
    # Initialization
    wait_start_master_trigger_msgs = ['left_lift', 'right_lift', 'both_lift',
                                      'left_backward', 'right_backward', 'both_backward',
                                      'left_center', 'right_center', 'both_center',
                                      'left_forward', 'right_forward', 'both_forward',
                                      'sweep_clockwise', 'sweep_counterclockwise']
    wait_start = WaitForMasterCmd(from_master, to_master, 'done', wait_start_master_trigger_msgs, ros_rate, 'wait_start')

    # Wait Orders
    wait_orders_master_trigger_msgs = list(wait_start_master_trigger_msgs)
    wait_orders_master_trigger_msgs += ['left_shallow_backward', 'right_shallow_backward', 'both_shallow_backward',
                                        'left_shallow_center', 'right_shallow_center', 'both_shallow_center',
                                        'left_shallow_forward', 'right_shallow_forward', 'both_shallow_forward',
                                        'sweep_shallow_clockwise', 'sweep_shallow_counterclockwise']
    wait_orders =  WaitForMasterCmd(from_master, to_master, 'done', wait_orders_master_trigger_msgs, ros_rate, 'wait_orders')

    # Send Leg Cmds - send cmds to one or both legs  #TODO: Clean up with dict
    left_lift = SendLegCmds(to_right_leg, to_left_leg, None, 'lift', ros_rate, 'left_lift')
    right_lift = SendLegCmds(to_right_leg, to_left_leg, 'lift', None, ros_rate, 'right_lift')
    both_lift = SendLegCmds(to_right_leg, to_left_leg, 'lift', 'lift', ros_rate, 'both_lift')

    left_backward = SendLegCmds(to_right_leg, to_left_leg, None, 'backward', ros_rate, 'left_backward')
    right_backward = SendLegCmds(to_right_leg, to_left_leg, 'backward', None, ros_rate, 'right_backward')
    both_backward = SendLegCmds(to_right_leg, to_left_leg, 'backward', 'backward', ros_rate, 'both_backward')

    left_center = SendLegCmds(to_right_leg, to_left_leg, None, 'center', ros_rate, 'left_center')
    right_center = SendLegCmds(to_right_leg, to_left_leg, 'center', None, ros_rate, 'right_center')
    both_center = SendLegCmds(to_right_leg, to_left_leg, 'center', 'center', ros_rate, 'both_center')

    left_forward = SendLegCmds(to_right_leg, to_left_leg, None, 'forward', ros_rate, 'left_forward')
    right_forward = SendLegCmds(to_right_leg, to_left_leg, 'forward', None, ros_rate, 'right_forward')
    both_forward = SendLegCmds(to_right_leg, to_left_leg, 'forward', 'forward', ros_rate, 'both_forward')

    sweep_clockwise = SendLegCmds(to_right_leg, to_left_leg, 'backward', 'forward', ros_rate, 'sweep_clockwise')
    sweep_counterclockwise = SendLegCmds(to_right_leg, to_left_leg, 'forward', 'backward', ros_rate, 'sweep_counterclockwise')

    left_shallow_backward = SendLegCmds(to_right_leg, to_left_leg, None, 'backward_shallow', ros_rate, 'left_shallow_backward')
    right_shallow_backward = SendLegCmds(to_right_leg, to_left_leg, 'backward_shallow', None, ros_rate, 'right_shallow_backward')
    both_shallow_backward = SendLegCmds(to_right_leg, to_left_leg, 'backward_shallow', 'backward_shallow', ros_rate, 'both_shallow_backward')

    left_shallow_center = SendLegCmds(to_right_leg, to_left_leg, None,  'center_shallow', ros_rate, 'left_shallow_center')
    right_shallow_center = SendLegCmds(to_right_leg, to_left_leg, 'center_shallow', None, ros_rate, 'right_shallow_center')
    both_shallow_center = SendLegCmds(to_right_leg, to_left_leg, 'center_shallow', 'center_shallow', ros_rate, 'both_shallow_center')

    left_shallow_forward = SendLegCmds(to_right_leg, to_left_leg, None, 'forward_shallow', ros_rate, 'left_shallow_forward')
    right_shallow_forward = SendLegCmds(to_right_leg, to_left_leg, 'forward_shallow', None, ros_rate, 'right_shallow_forward')
    both_shallow_forward = SendLegCmds(to_right_leg, to_left_leg, 'forward_shallow', 'forward_shallow', ros_rate, 'both_shallow_forward')

    sweep_shallow_clockwise = SendLegCmds(to_right_leg, to_left_leg, 'backward_shallow', 'forward_shallow', ros_rate, 'sweep_shallow_clockwise')
    sweep_shallow_counterclockwise = SendLegCmds(to_right_leg, to_left_leg, 'forward_shallow', 'backward_shallow', ros_rate, 'sweep_shallow_counterclockwise')

    # Sync - wait until one or both legs return a 'done' msg
    wait_left = SyncLegs(from_right_leg, from_left_leg, None, 'done', ros_rate, 'wait_left')
    wait_right = SyncLegs(from_right_leg, from_left_leg, 'done', None, ros_rate, 'wait_right')
    wait_both = SyncLegs(from_right_leg, from_left_leg, 'done', 'done', ros_rate, 'wait_both')

    #---------------------------------------------------------------------------
    # Open SMACH state machine
    with segment_fsm_node:  #TODO: Clean up with dict
        wait_start_transitions = {msg: msg.upper() for msg in wait_start_master_trigger_msgs}
        wait_start_transitions['failure'] = 'WAIT_START'
        StateMachine.add('WAIT_START', wait_start, transitions=wait_start_transitions)
        wait_orders_transitions = {msg: msg.upper() for msg in wait_orders_master_trigger_msgs}
        wait_orders_transitions['failure'] = 'WAIT_ORDERS'
        StateMachine.add('WAIT_ORDERS', wait_orders, transitions=wait_orders_transitions)
        StateMachine.add('LEFT_LIFT', left_lift, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_LIFT', right_lift, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_LIFT', both_lift, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_BACKWARD', left_backward, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_BACKWARD', right_backward, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_BACKWARD', both_backward, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_CENTER', left_center, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_CENTER', right_center, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_CENTER', both_center, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_FORWARD', left_forward, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_FORWARD', right_forward, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_FORWARD', both_forward, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('SWEEP_CLOCKWISE', sweep_clockwise, transitions={'success':'WAIT_BOTH'})
        StateMachine.add('SWEEP_COUNTERCLOCKWISE', sweep_counterclockwise, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_SHALLOW_BACKWARD', left_shallow_backward, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_SHALLOW_BACKWARD', right_shallow_backward, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_SHALLOW_BACKWARD', both_shallow_backward, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_SHALLOW_CENTER', left_shallow_center, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_SHALLOW_CENTER', right_shallow_center, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_SHALLOW_CENTER', both_shallow_center, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('LEFT_SHALLOW_FORWARD', left_shallow_forward, transitions={'success':'WAIT_LEFT'})
        StateMachine.add('RIGHT_SHALLOW_FORWARD', right_shallow_forward, transitions={'success':'WAIT_RIGHT'})
        StateMachine.add('BOTH_SHALLOW_FORWARD', both_shallow_forward, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('SWEEP_SHALLOW_CLOCKWISE', sweep_shallow_clockwise, transitions={'success':'WAIT_BOTH'})
        StateMachine.add('SWEEP_SHALLOW_COUNTERCLOCKWISE', sweep_shallow_counterclockwise, transitions={'success':'WAIT_BOTH'})

        StateMachine.add('WAIT_LEFT', wait_left, transitions={'success':'WAIT_ORDERS', 'failure':'WAIT_LEFT'})
        StateMachine.add('WAIT_RIGHT', wait_right, transitions={'success':'WAIT_ORDERS', 'failure':'WAIT_RIGHT'})
        StateMachine.add('WAIT_BOTH', wait_both, transitions={'success':'WAIT_ORDERS', 'failure':'WAIT_BOTH'})

    #---------------------------------------------------------------------------
    # Create and start the introspection server - for visualization / debugging
    # $ sudo apt-get install ros-kinetic-smach-viewer
    # $ rosrun smach_viewer smach_viewer.py
    sis = smach_ros.IntrospectionServer('segment_fsm_node' + str(rospy.get_name()), segment_fsm_node, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: segment_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing  - type: segment_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = segment_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
