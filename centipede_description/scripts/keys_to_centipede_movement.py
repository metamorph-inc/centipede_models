#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: keys_to_centipede_movement.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 12/06/2017
# Edit Date: 12/06/2017
#
# Description:
# ROS node that listens to keystrokes published
#   on the /keystroke topic,
# and sends movement commands
#   to the '/cmd_centipede_move' topic.
#
'''

import sys
import argparse
import rospy
from std_msgs.msg import String


def parse_args(args):
    """Parses a list of arguments using argparse

    Args:
        args (list): Command line arguments (sys[1:]).
    Returns:
        obj: An argparse ArgumentParser() instance
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-ros_rate', type=float, default=100, help="type=float, Description='rate at which ROS node publishes'")
    return parser.parse_args(args)


class KeysToCentipedeMovement(object):
    """Subscribes to /keystroke topic and publishes to /cmd_centipede_move topic

    Attributes:
        ros_rate    (float)
    """
    def __init__(self, ros_rate):
        self._rate = rospy.Rate(ros_rate)
        self._key_sub = rospy.Subscriber('keystroke', String, self._key_cb)
        self._cmd_pub = rospy.Publisher('cmd_centipede_move', String, queue_size=1)
        self._crouch = False
        self._mode = 'stop'
        self._direction = ''
        self._keypress = None

    def _key_cb(self, msg):
        """Callback function
        """
        self._keypress = msg.data

    def _process_keypress(self):
        """Maps a received keypress to a movement cmd for the centipede robot

        Returns:
            str: movement cmd for to be published to centipede
        """
        mode_mapping = {'x':'stop', 'w':'walk', 's':'walk', 'a':'rotate', 'd':'rotate'}
        direction_mapping = {'x':'', 'w':'_forward', 's':'_reverse', 'a':'_left', 'd':'_right'}
        crouch_mapping = {True:'_shallow', False:''}
        kp = self._keypress
        cmd = None
        if kp == 'e' or kp == 'c':
            if kp == 'e':
                self._crouch = False
                print('crouch = False')
            else:
                self._crouch = True
                print('crouch = True')
            if self._mode == 'stop':
                cmd = self._mode
            else:
                cmd = self._mode + crouch_mapping[self._crouch] + self._direction
            print('cmd: '+cmd)
            return cmd
        elif kp in mode_mapping:
            self._mode = mode_mapping[kp]
            self._direction = direction_mapping[kp]
            if kp == 'x':
                self._mode = mode_mapping[kp]
                cmd = mode_mapping[kp]
            else:
                self._mode = mode_mapping[kp]
                cmd = mode_mapping[kp] + crouch_mapping[self._crouch] \
                    + direction_mapping[kp]
            print('cmd: '+cmd)
            return cmd
        else:
            print('Keypress not mapped to any behavior!!!')
            return None

    def execute(self):
        """rospy loop - publishes movement cmds to /cmd_centipede_move topic
        """
        cmd = None
        prev_cmd = None
        while not rospy.is_shutdown():
            if self._keypress is not None:
                cmd = self._process_keypress()
            if cmd is not None:
                self._cmd_pub.publish(cmd)
                prev_cmd = cmd
            elif prev_cmd is not None:
                self._cmd_pub.publish(prev_cmd)
            else:
                pass  # do nothing
            self._keypress = None
            self._rate.sleep()


if __name__ == '__main__':
    # ROS stuff
    rospy.init_node('keys_to_centipede_movement')

    # Parse cmd line args
    args = rospy.myargv(argv=sys.argv)[1:]
    parser = parse_args(args)
    ros_rate = parser.ros_rate

    # Create KeysToCentipedeMovement object
    keys_to_move = KeysToCentipedeMovement(ros_rate)

    # Start
    keys_to_move.execute()
