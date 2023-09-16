# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
from gol import *
from ros_interface import pubLiTouchCmd
from components import getComponentByName, getComponentByCid


def interP():
    print(213)

def getComponentbyName(name):
    return getComponentByName(name)
def getComponentvbyCid(cid):
    return getComponentByCid(cid)

def flashInfo(info):
    gol.state_bar.flash( info )

def flashWarning(info):
    gol.state_bar.flash( info, "warning" )

def flashError(info):
    gol.state_bar.flash( info, "error" )

def flashGood(info):
    gol.state_bar.flash( info, "good" )