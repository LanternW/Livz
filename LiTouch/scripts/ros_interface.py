#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

from components import getComponentByName, MiniTerminal

livz_touch_publisher    = None

terminal_msg_subscriber = None

def pubLiTouchCmd(cmd, param_list = []):
    global livz_touch_publisher
    msg      = Float64MultiArray()
    msg.data = [cmd]
    for item in param_list:
        msg.data.append(item)
    livz_touch_publisher.publish(msg)


def terminalMsgCallback(msg):
    raw_str = msg.data 
    first_str, second_str = raw_str.split('#', 1) 
    terminal_component = getComponentByName(first_str)
    if terminal_component == None:
        return 
    if isinstance( terminal_component , MiniTerminal ):
        terminal_component.feedString(second_str)


################################################
######################   INIT   ################
################################################
def initRosInterface():
    global livz_touch_publisher, terminal_msg_subscriber
    rospy.init_node("litouch_node" ,anonymous=True)
    livz_touch_publisher    = rospy.Publisher('/litouch_cmd', Float64MultiArray, queue_size=10)
    terminal_msg_subscriber = rospy.Subscriber('/litouch_terminal_msg', String, terminalMsgCallback)


    # 获取所有节点信息
    node_dict = rospy.get_node_uri()
    print(node_dict)






