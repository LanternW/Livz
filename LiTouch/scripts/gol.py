#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import os
import pygame
pygame.init()
pygame.font.init()

def print_color(text, color="white"):
    colors = {
        "black": "\033[30m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "purple": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "reset": "\033[0m"
    }

    if color not in colors:
        color = "white" # 默认使用白色

    print(f"{colors[color]}{text}{colors['reset']}")

class Gol():
    def __init__(self) -> None:
        self.VERSION_INFO = "v3.0.0"

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.DEFAULT_CONFIG_PATH = os.path.join(script_dir, "../config/default.json")
        self.CONFIG_PATH         = self.DEFAULT_CONFIG_PATH

        self.DEFAULT_CALLBACK_PATH = "config/default_action.py"
        self.CALLBACK_PATH         = self.DEFAULT_CALLBACK_PATH

        self.REL_CBK_PATH = ""

        self.TEMPLATE_CONFIG_PATH   = os.path.join(script_dir, "../config/layout_template.json")
        self.TEMPLATE_CALLBACK_PATH = os.path.join(script_dir, "../config/action_template.py")
        self.FONT_PATH              = os.path.join(script_dir, "./fonts/Monaco-1.ttf")
        
        self.APP_WIDTH   = 1080
        self.APP_HEIGHT  = 640
        self.MENU_HEIGHT = 40
        self.COMPONENT_Y = self.MENU_HEIGHT
        self.STATE_Y     = self.APP_HEIGHT - self.MENU_HEIGHT
        self.BASE_WINDOW      = pygame.display.set_mode( (60, 80) )  
        self.MENU_WINDOW      = None #pygame.Surface((60, 20),  pygame.SRCALPHA) 
        self.STATE_WINDOW     = None #pygame.Surface((60, 20),  pygame.SRCALPHA)        
        self.COMPONENT_WINDOW = None #pygame.Surface((60, 40),  pygame.SRCALPHA) 

        #辅助
        self.MOUSEBUTTON_LEFT      = 1
        self.MOUSEBUTTON_MID       = 2
        self.MOUSEBUTTON_RIGHT     = 3
        self.MOUSEBUTTON_WHEELUP   = 4
        self.MOUSEBUTTON_WHEELDOWN = 5
        
        self.COLOR_YELLOW  = (255,255,120,255)
        self.COLOR_WHITE   = (255,255,255,255)
        self.COLOR_GREEN   = (0,255,0,255)
        self.COLOR_BLUE    = (100,100,255,255)
        self.COLOR_ORANGE  = (255,120,0,255)
        self.COLOR_BLACK   = (0,0,0,255)
        self.COLOR_RED     = (255,50,50,255)
        
        self.COLOR_DARK_WHITE  = (100,100,100,255)
        self.COLOR_DARK_RED    = (100,0,0,255)
        self.COLOR_DARK_ORANGE = (100,70,0,255)
        self.COLOR_DARK_GREEN  = (0,100,0,255)

        self.COLOR_SKY_BLUE    = (100,150,255,255)
        
        self.BASE_WINDOW_COLOR = (27,27,28,255)
        self.INFO_WINDOW_COLOR = (37,37,38,255)
        self.ITEM_WINDOW_COLOR = (30,30,30,255)
        self.STATE_WINDOW_COLOR = (10,10,10,255)
        self.MENU_WINDOW_COLOR = (15,15,15,255)
        self.COMPONENT_WINDOW_COLOR = (27,27,28,255)
        self.COMPONENT_COLOR = (17,17,18,255)
        self.STACK_LINE_COLOR   = (130,130,255,255)
        self.GROUP_BORDER_COLOR = (255,255,255,255)
        self.DB_BORDER_COLOR    = (155,155,0,255)
        self.SCROLL_BORDER_COLOR = (57,57,57,255)
        self.SCROLL_HOVER_COLOR  = (70,70,70,255)
        self.SWITCH_BAR_COLOR = (35,35,35,255)
        self.SWITCH_BORDER_COLOR = (75,75,75,255)
        self.SPARE_SWITCH_COLOR       = (30,155,30,255)
        self.SPARE_SWITCH_COLOR_HOVER = (60,195,60,255)
        self.LENT_SWITCH_COLOR       = (155,155,30,255)
        self.LENT_SWITCH_COLOR_HOVER = (195,195,60,255)
        self.SPARE_FONT_COLOR = (154, 255, 164, 255)
        self.LENT_FONT_COLOR  = (255, 238, 140, 255)
        self.FONT_COLOR        = (255,255,255,255)
        self.ITEM_COLOR        = (20,20,40,255)
        self.ITEM_COLOR_HOVER  = (40,40,80,255)
        self.ITEM_COLOR_CHOSEN = (130,100,20,255)
        self.TAG_COLOR         = (20,20,20,255)
        self.TAG_COLOR_HOVER   = (60,60,60,255)
        self.SEL_COLOR         = (50,50,50,255)
        self.SEL_COLOR_HOVER   = (50,50,100,255)
        self.SEL_COLOR_CUSTOM  = (250,200,100,255)

        self.fonts  = {}
        self.mono_fonts  = {}
        self.initialize_fonts()

        self.state_bar  = None
        self.states     = []
        self.menu       = []
        self.components = []
    
    def initialize_fonts(self):
        # 40个常用字号，可以根据需要调整这个列表
        common_font_sizes = [8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86]
        for size in common_font_sizes:
            self.fonts[size]      = pygame.font.Font(None, size)
            self.mono_fonts[size] = pygame.font.Font(self.FONT_PATH, size)
    
    def getFont(self, size):
        closest_size = min(self.fonts.keys(), key=lambda x: abs(x - size))
        return self.fonts[closest_size], closest_size
    
    def getMonoFont(self, size):
        closest_size = min(self.fonts.keys(), key=lambda x: abs(x - size))
        return self.mono_fonts[closest_size], closest_size

                

gol = Gol()


