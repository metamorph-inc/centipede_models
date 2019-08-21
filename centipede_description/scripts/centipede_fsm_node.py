#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: centipede_fsm_node.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 11/29/2017
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


class SendSegmentCmd(State):
    """SMACH state: Sends a cmd to a segment and waits for a response

    Attributes:
        segment_pub_topic   (str): publish to segment topic
        segment_cmd         (str): message published on segment_pub_topic
        segment_sub_topic   (str): subscribe to segment_sub_topic
        segment_success_msg (str): recognized success message from segment
        segment_failure_msg (str): recognized failure message from segment
        ros_rate            (float)
        instance_name       (str)
    """
    def __init__(self, segment_pub_topic, segment_cmd, segment_sub_topic,
                 segment_success_msg, segment_failure_msg, ros_rate, instance_name):
        State.__init__(self, outcomes=['success', 'failure'])
        # ROS stuff
        self.segment_pub = rospy.Publisher(segment_pub_topic, String, queue_size=1)
        self.segment_cmd = segment_cmd
        self.segment_sub = rospy.Subscriber(segment_sub_topic, String, self.segment_callback)
        self.segment_success_msg = segment_success_msg
        self.segment_failure_msg = segment_failure_msg
        self.rate = rospy.Rate(ros_rate)
        self.instance_name = instance_name
        self.active_flag = False
        self.msg_from_segment = None

    def execute(self, userdata):
        self.active_flag = True
        # Send a message down the chain
        if self.segment_cmd is not None:
            print("State: " + self.instance_name + ", Sent sub_msg: " + self.segment_cmd + " to segment...")
            self.segment_pub.publish(self.segment_cmd)
        if (self.segment_success_msg is None):
            return 'success'
        else:  # wait for a response to come back up chain - could add timeout here
            while not rospy.is_shutdown():
                if self.msg_from_segment is not None:
                    if (self.msg_from_segment == self.segment_success_msg):
                        self.clean_up()
                        return 'success'
                    elif (self.msg_from_segment == self.segment_failure_msg):
                        self.clean_up()
                        return 'failure'
                    else:
                        print("Unrecognized sub_msg: " + self.msg_from_segment)
                    self.msg_from_segment = None
                self.rate.sleep()
            return 'failure'  # if rospy.is_shutdown()

    def segment_callback(self, msg):
        if self.active_flag:
            print("State: " + self.instance_name + ", Received sub_msg: " + msg.data)
            self.msg_from_segment = msg.data

    def clean_up(self):
        self.active_flag = False
        self.msg_from_segment = None


class WaitForMasterCmd(State):
    """SMACH state: Waits for a trigger message from a master FSM

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
                    print("Unrecognized msg_from_master: " + self.msg_from_master)
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
    parser.add_argument('-num_segments', type=int, default=2, help="type=int, default=2, Description='number of segments'")
    parser.add_argument('-from_segment_0', type=str, required=True, help="type=str, Description='segment ROS topic to subscribe to'")
    parser.add_argument('-to_segment_0', type=str, required=True, help="type=str, Description='segment ROS topic to publish to'")
    parser.add_argument('-from_segment_1', type=str, required=True, help="type=str, Description='segment ROS topic to subscribe to'")
    parser.add_argument('-to_segment_1', type=str, required=True, help="type=str, Description='segment ROS topic to publish to'")
    max_segments = 20
    for i in range(2, max_segments, 1):
        parser.add_argument('-from_segment_'+str(i), type=str, help="type=str, Description='segment ROS topic to subscribe to'")
        parser.add_argument('-to_segment_'+str(i), type=str, help="type=str, Description='segment ROS topic to publish to'")
    parser.add_argument('-from_master', type=str, required=True, help="type=str, Description='master ROS topic to subscribe to'")
    parser.add_argument('-to_master', type=str, required=True, help="type=str, Description='master ROS topic to publish to'")
    parser.add_argument('-ros_rate', type=float, default=100.0, help="type=float, default=100.0'")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def build_concurrence_container(outcomes, default_outcome, outcome_map, concurrent_states):
    """Create & returns a SMACH Concurrence instance from a dictionary of SMACH states
    Args:
        outcomes            (list of str)
        default_outcome     (str)
        outcome_map         (dict of dict)
        concurrent_states   (dict of class instances)
    Returns:
        dict: A dict with instance names as keys and class instances as values
    """
    concurrence = Concurrence(outcomes=outcomes, default_outcome=default_outcome,
        outcome_map=outcome_map)
    with concurrence:
        for name, instance in concurrent_states.items():
            Concurrence.add(name.upper(), instance)
    return concurrence


def build_sendsegmentcmd_mode(num_segments, cent_to_seg_dict,
    seg_to_cent_dict, step_to_seg_mapping, ros_rate):
    """Creates and returns a dictionary of SMACH Concurrence containers
    Args:
        num_segments        (int): expects int >= 2
        cent_to_seg_dict    (dict of str): expects keys 'seg0', 'seg1', 'seg2'...
        seg_to_cent_dict    (dict of str): expects keys 'seg0', 'seg1', 'seg2'...
        step_to_seg_mapping (dict of tuple): expects 'step_name':('even_segment_cmd','odd_segment_cmd')
    Returns:
        dict
    """
    concurrence_dict = {}
    for step, cmds in step_to_seg_mapping.items():
        seg_cmds = [cmds[0] if i % 2 == 0 else cmds[1] for i in range(num_segments)]
        concurrent_states = {}
        for i in range(num_segments):  # create a state instance for each segment
            state = SendSegmentCmd(cent_to_seg_dict['seg'+str(i)], seg_cmds[i],
                seg_to_cent_dict['seg'+str(i)], "done", None, ros_rate, 'seg'+str(i))
            concurrent_states['seg'+str(i)] = state
        outcome_map = {'success':{state_name.upper():'success' for state_name in concurrent_states}}
        concurrence = build_concurrence_container(['success', 'failure'], 'failure',
            outcome_map, concurrent_states)
        concurrence_dict[step] = concurrence  # add concurrent instance to mode dict
    return concurrence_dict


def main():
    rospy.init_node('centipede_fsm_node', anonymous=True)

    parser = parse_args(sys.argv[1:])
    args_dict = vars(parser)
    num_segments = parser.num_segments
    seg_to_cent_dict = {}  # segment_to_centipede
    cent_to_seg_dict = {}  # centipede_to_segment
    for i in range(num_segments):
        seg_to_cent_dict['seg' + str(i)] = args_dict['from_segment_'+str(i)]
        cent_to_seg_dict['seg' + str(i)] = args_dict['to_segment_'+str(i)]
    from_master = parser.from_master
    to_master = parser.to_master
    ros_rate = parser.ros_rate

    # Start mode
    wait_start_master_trigger_msgs = ['stop',
                                      'walk_forward', 'walk_reverse',
                                      'rotate_left', 'rotate_right']
                                      #TODO: Add more master msgs here
    wait_start = WaitForMasterCmd(from_master, to_master, 'done',
        wait_start_master_trigger_msgs, ros_rate, 'wait_start')

    # Wait Orders mode
    wait_orders_master_trigger_msgs = list(wait_start_master_trigger_msgs) \
                                        + ['walk_shallow_forward', 'walk_shallow_reverse',
                                           'rotate_shallow_left', 'rotate_shallow_right']
    wait_orders = WaitForMasterCmd(from_master, to_master, 'done',
    wait_orders_master_trigger_msgs, ros_rate, 'wait_orders')

    # Walking Forward mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_forward','right_forward'),
                           'b':('right_lift','left_lift'),
                           'c':('left_backward','right_backward'),
                           'd':('right_forward','left_forward'),
                           'e':('left_lift','right_lift'),
                           'end':('right_backward','left_backward')}
    walk_forward = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Walking Reverse mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_backward','right_backward'),
                           'b':('right_lift','left_lift'),
                           'c':('left_forward','right_forward'),
                           'd':('right_backward','left_backward'),
                           'e':('left_lift','right_lift'),
                           'end':('right_forward','left_forward')}
    walk_reverse = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Rotate Left mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_backward','right_forward'),
                           'b':('right_lift','left_lift'),
                           'c':('right_forward','left_backward'),
                           'd':('sweep_clockwise','sweep_clockwise'),
                           'e':('left_lift','right_lift'),
                           'end':('left_center','right_center')}
    rotate_left = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Rotate Right mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_forward','right_backward'),
                           'b':('right_lift','left_lift'),
                           'c':('right_backward','left_forward'),
                           'd':('sweep_counterclockwise','sweep_counterclockwise'),
                           'e':('left_lift','right_lift'),
                           'end':('left_center','right_center')}
    rotate_right = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Walking Shallow Forward mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_shallow_forward','right_shallow_forward'),
                           'b':('right_lift','left_lift'),
                           'c':('left_shallow_backward','right_shallow_backward'),
                           'd':('right_shallow_forward','left_shallow_forward'),
                           'e':('left_lift','right_lift'),
                           'end':('right_shallow_backward','left_shallow_backward')}
    walk_shallow_forward = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Walking Shallow Reverse mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_shallow_backward','right_shallow_backward'),
                           'b':('right_lift','left_lift'),
                           'c':('left_shallow_forward','right_shallow_forward'),
                           'd':('right_shallow_backward','left_shallow_backward'),
                           'e':('left_lift','right_lift'),
                           'end':('right_shallow_forward','left_shallow_forward')}
    walk_shallow_reverse = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Rotate Shallow Left mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_shallow_backward','right_shallow_forward'),
                           'b':('right_lift','left_lift'),
                           'c':('right_shallow_forward','left_shallow_backward'),
                           'd':('sweep_shallow_clockwise','sweep_shallow_clockwise'),
                           'e':('left_lift','right_lift'),
                           'end':('left_shallow_center','right_shallow_center')}
    rotate_shallow_left = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    # Rotate Shallow Right mode
    step_to_seg_mapping = {'start':('left_lift','right_lift'),  # 'step_name':('even_segment_cmd','odd_segment_cmd')
                           'a':('left_shallow_forward','right_shallow_backward'),
                           'b':('right_lift','left_lift'),
                           'c':('right_shallow_backward','left_shallow_forward'),
                           'd':('sweep_shallow_counterclockwise','sweep_shallow_counterclockwise'),
                           'e':('left_lift','right_lift'),
                           'end':('left_shallow_center','right_shallow_center')}
    rotate_shallow_right = build_sendsegmentcmd_mode(num_segments,
        cent_to_seg_dict, seg_to_cent_dict, step_to_seg_mapping, ros_rate)

    #TODO: Add more modes here

    #---------------------------------------------------------------------------
    # Create & open SMACH state machine
    centipede_fsm_node = StateMachine(outcomes=['success'])
    with centipede_fsm_node:
        StateMachine.add('WAIT_START', wait_start,
            transitions={'failure':'WAIT_START', 'stop':'WAIT_START',
                         'walk_forward':'WALK_FORWARD_START',
                         'walk_reverse':'WALK_REVERSE_START',
                         'rotate_left':'ROTATE_LEFT_START',
                         'rotate_right':'ROTATE_RIGHT_START'
                         })
        StateMachine.add('WAIT_ORDERS', wait_orders,
            transitions={'failure':'WAIT_ORDERS', 'stop':'WAIT_ORDERS',
                         'walk_forward':'WALK_FORWARD_START',
                         'walk_reverse':'WALK_REVERSE_START',
                         'rotate_left':'ROTATE_LEFT_START',
                         'rotate_right':'ROTATE_RIGHT_START',
                         'walk_shallow_forward':'WALK_SHALLOW_FORWARD_START',
                         'walk_shallow_reverse':'WALK_SHALLOW_REVERSE_START',
                         'rotate_shallow_left':'ROTATE_SHALLOW_LEFT_START',
                         'rotate_shallow_right':'ROTATE_SHALLOW_RIGHT_START'
                         })

        order = ['start', 'a', 'b', 'c', 'd', 'e', 'end']
        for i in range(len(order)-1):
            StateMachine.add('WALK_FORWARD_'+order[i].upper(), walk_forward[order[i]],
                transitions={'failure':'WALK_FORWARD_'+order[i].upper(), 'success':'WALK_FORWARD_'+order[i+1].upper()})
        StateMachine.add('WALK_FORWARD_'+order[i+1].upper(), walk_forward['end'],
            transitions={'failure':'WALK_FORWARD_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('WALK_REVERSE_'+order[i].upper(), walk_reverse[order[i]],
                transitions={'failure':'WALK_REVERSE_'+order[i].upper(), 'success':'WALK_REVERSE_'+order[i+1].upper()})
        StateMachine.add('WALK_REVERSE_'+order[i+1].upper(), walk_reverse['end'],
            transitions={'failure':'WALK_REVERSE_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('ROTATE_LEFT_'+order[i].upper(), rotate_left[order[i]],
                transitions={'failure':'ROTATE_LEFT_'+order[i].upper(), 'success':'ROTATE_LEFT_'+order[i+1].upper()})
        StateMachine.add('ROTATE_LEFT_'+order[i+1].upper(), rotate_left['end'],
            transitions={'failure':'ROTATE_LEFT_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('ROTATE_RIGHT_'+order[i].upper(), rotate_right[order[i]],
                transitions={'failure':'ROTATE_RIGHT_'+order[i].upper(), 'success':'ROTATE_RIGHT_'+order[i+1].upper()})
        StateMachine.add('ROTATE_RIGHT_'+order[i+1].upper(), rotate_right['end'],
            transitions={'failure':'ROTATE_RIGHT_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('WALK_SHALLOW_FORWARD_'+order[i].upper(), walk_shallow_forward[order[i]],
                transitions={'failure':'WALK_SHALLOW_FORWARD_'+order[i].upper(), 'success':'WALK_SHALLOW_FORWARD_'+order[i+1].upper()})
        StateMachine.add('WALK_SHALLOW_FORWARD_'+order[i+1].upper(), walk_shallow_forward['end'],
            transitions={'failure':'WALK_SHALLOW_FORWARD_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('WALK_SHALLOW_REVERSE_'+order[i].upper(), walk_shallow_reverse[order[i]],
                transitions={'failure':'WALK_SHALLOW_REVERSE_'+order[i].upper(), 'success':'WALK_SHALLOW_REVERSE_'+order[i+1].upper()})
        StateMachine.add('WALK_SHALLOW_REVERSE_'+order[i+1].upper(), walk_shallow_reverse['end'],
            transitions={'failure':'WALK_SHALLOW_REVERSE_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('ROTATE_SHALLOW_LEFT_'+order[i].upper(), rotate_shallow_left[order[i]],
                transitions={'failure':'ROTATE_SHALLOW_LEFT_'+order[i].upper(), 'success':'ROTATE_SHALLOW_LEFT_'+order[i+1].upper()})
        StateMachine.add('ROTATE_SHALLOW_LEFT_'+order[i+1].upper(), rotate_shallow_left['end'],
            transitions={'failure':'ROTATE_SHALLOW_LEFT_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        for i in range(len(order)-1):
            StateMachine.add('ROTATE_SHALLOW_RIGHT_'+order[i].upper(), rotate_shallow_right[order[i]],
                transitions={'failure':'ROTATE_SHALLOW_RIGHT_'+order[i].upper(), 'success':'ROTATE_SHALLOW_RIGHT_'+order[i+1].upper()})
        StateMachine.add('ROTATE_SHALLOW_RIGHT_'+order[i+1].upper(), rotate_shallow_right['end'],
            transitions={'failure':'ROTATE_SHALLOW_RIGHT_'+order[i+1].upper(), 'success':'WAIT_ORDERS'})

        #TODO: Add more concurrent SMACH state instances here

    #---------------------------------------------------------------------------
    # Create and start the introspection server - for visualization / debugging
    sis = smach_ros.IntrospectionServer('centipede_fsm_node' + str(rospy.get_name()), centipede_fsm_node, '/SM_ROOT')#+ str(rospy.get_name()))
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing - type: centipede_fsm_node.py | node: " + str(rospy.get_name()) + "\n")

    # Execute SMACH state machine
    print("Input received. Executing  - type: centipede_fsm_node.py | node: " + str(rospy.get_name()) + "...\n")
    outcome = centipede_fsm_node.execute()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
