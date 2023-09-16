#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
from gol import gol
import rospy
from ros_interface import initRosInterface
import pygame

from handle import *
from painter import painter
from event   import event_handler


checkAndReadConfig()
init()
initRosInterface()

while True: 
    events = pygame.event.get()
    for event in events: 
        event_handler.eventHandle( event )

    painter.renderScreen()
    tick(dt = 0.01)

    pygame.display.flip()

pygame.quit()  
