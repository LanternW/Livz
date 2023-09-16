#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.

from gol import gol, print_color
import pygame
import sys
from input import input_manager
from utils import *



class EventHandler():
    def __init__(self):
        self.MOUSE_IN_MENU      = 1
        self.MOUSE_IN_COMPONENT = 2
        self.MOUSE_IN_STATE     = 3

        self.mouse_in_which_window = 0
    
    def relativeMouseCoord(self, x,y):
        window_pos_x = 0
        window_pos_y = 0
        mouse_in_window = False
        self.mouse_in_which_window = 0 
        if isMouseInWindow( x,y, (0,0), (gol.APP_WIDTH, gol.MENU_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = 0
            self.mouse_in_which_window = self.MOUSE_IN_MENU

        elif isMouseInWindow( x,y, (0, gol.COMPONENT_Y), (gol.APP_WIDTH, gol.APP_HEIGHT - 2*gol.MENU_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = gol.COMPONENT_Y
            self.mouse_in_which_window = self.MOUSE_IN_COMPONENT

        elif isMouseInWindow( x,y, (0, gol.STATE_Y), (gol.APP_WIDTH, gol.MENU_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = gol.STATE_Y
            self.mouse_in_which_window = self.MOUSE_IN_STATE
        
        relative_x = x - window_pos_x
        relative_y = y - window_pos_y
        return (relative_x , relative_y )


    def eventHandle(self, e ):
        input_manager.inputEventCallback( e )
        if e.type == pygame.KEYDOWN:
            self.keyDownHandle( e.key , e.unicode)
        if e.type == pygame.MOUSEMOTION:
            self.mouseMotionHandle(e.pos[0], e.pos[1])
        if e.type == pygame.MOUSEBUTTONDOWN:
            self.mouseClickHandle( e.button )
        if e.type == pygame.MOUSEBUTTONUP:
            self.mouseUpHandle( e.button )
        if e.type == pygame.QUIT:
            print_color("[LiTouch] 退出LiTouch", "red")
            sys.exit()
            
    def mouseMotionHandle(self, x,y):
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_MENU:
            for component in gol.menu:
                component.onMouseMotion( rel_x, rel_y )
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            for component in gol.components:
                component.onMouseMotion( rel_x, rel_y )
        

    def mouseClickHandle(self, btn ):
        x, y = pygame.mouse.get_pos()
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_MENU:
            for component in gol.menu:
                component.onMouseClick( rel_x, rel_y, btn )
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            for component in gol.components:
                component.onMouseClick( rel_x, rel_y, btn )
    
    def mouseUpHandle(self, btn ):
        x, y = pygame.mouse.get_pos()
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_MENU:
            for component in gol.menu:
                component.onMouseUp( rel_x, rel_y, btn )
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            for component in gol.components:
                component.onMouseUp( rel_x, rel_y, btn )

    def keyDownHandle(self, key, unicode):
        if key == pygame.K_ESCAPE: # test
            print_color("[LiTouch] 退出LiTouch", "red")
            sys.exit()

        for component in gol.components:
            component.onKeyInput( key , unicode)


event_handler = EventHandler()