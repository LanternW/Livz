#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
from gol import gol
import pygame

class Painter:
    def __init__(self):
        pass

    def clearScreen(self):
        gol.BASE_WINDOW.fill(gol.BASE_WINDOW_COLOR)
        # gol.COMPONENT_WINDOW.fill(gol.COMPONENT_WINDOW_COLOR)
        gol.COMPONENT_WINDOW.blit(gol.COMPONENT_BACKGROUND, (0,0))
        gol.MENU_WINDOW.fill(gol.MENU_WINDOW_COLOR)
        gol.STATE_WINDOW.fill(gol.STATE_WINDOW_COLOR)

    def renderComponents(self):
        for component in gol.components:
            component.render( gol.COMPONENT_WINDOW )
        for component in gol.menu:
            component.render( gol.MENU_WINDOW )
        for component in gol.states:
            component.render( gol.STATE_WINDOW )

    def renderScreen(self):
        self.clearScreen()
        
        # 依次绘制各窗口
        self.renderComponents()

        gol.BASE_WINDOW.blit( gol.COMPONENT_WINDOW, (0, gol.COMPONENT_Y) )
        gol.BASE_WINDOW.blit( gol.STATE_WINDOW,     (0, gol.STATE_Y) )
        gol.BASE_WINDOW.blit( gol.MENU_WINDOW,      (0, 0) )

        pygame.display.update()
        pygame.time.wait(1)

painter = Painter() 
