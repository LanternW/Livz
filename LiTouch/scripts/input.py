#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import pygame
from gol import *
from utils import *

class InputManager():
    def __init__(self):
        self.down_push_buttons = []
        self.down_push_keys    = []
        self.mouse_x           = 0
        self.mouse_y           = 0
        self.mouse_x_vel       = 0
        self.mouse_y_vel       = 0
        self.intransition      = 0
    
    
    def absoluteMouseCoord(self):
        return (self.mouse_x ,self.mouse_y)
    
    def isKeyDown(self, key):
        if key in self.down_push_keys:
            return True
        else:
            return False
    
    def isButtonDown(self, button):
        if button in self.down_push_buttons:
            return True
        else:
            return False
    
    def inputEventCallback(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 5 or event.button == 4:
                return 
            if self.isButtonDown(event.button):
                return
            self.down_push_buttons.append(event.button)

        if event.type == pygame.MOUSEMOTION:
            self.mouse_x_vel = event.pos[0] - self.mouse_x
            self.mouse_y_vel = event.pos[1] - self.mouse_y
            self.mouse_x = event.pos[0]
            self.mouse_y = event.pos[1]
            return 

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 4 or event.button == 5:
                return 
            if self.isButtonDown(event.button):
                self.down_push_buttons.remove(event.button)
            return

        if event.type == pygame.KEYDOWN:  
            if self.isKeyDown(event.key):
                return
            self.down_push_keys.append(event.key)
            return

        if event.type == pygame.KEYUP:
            if self.isKeyDown(event.key):
                self.down_push_keys.remove(event.key)
            return 

input_manager = InputManager()