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
import numpy as np
pygame.init()
pygame.font.init()

def create_gradient_surface(width_, height_, start_color, end_color, dir="vertical"):
    width  = int(height_)
    height = int(width_)
    if dir == 'horizontal':
        gradient = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            color = (
                start_color[0] * (1 - y/height) + end_color[0] * y/height,
                start_color[1] * (1 - y/height) + end_color[1] * y/height,
                start_color[2] * (1 - y/height) + end_color[2] * y/height
            )
            gradient[y, :, :] = color
    elif dir == 'vertical':
        gradient = np.zeros((height, width, 3), dtype=np.uint8)
        for x in range(width):
            color = (
                start_color[0] * (1 - x/width) + end_color[0] * x/width,
                start_color[1] * (1 - x/width) + end_color[1] * x/width,
                start_color[2] * (1 - x/width) + end_color[2] * x/width
            )
            gradient[:, x, :] = color
    else:
        raise ValueError("Invalid direction. Expected 'vertical' or 'horizontal'.")
    return pygame.surfarray.make_surface(gradient)

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

        self.COLOR_TRANS = (244,225,176,0)
        self.COLOR_LIGHT_YELLOW_TRANS = (244,225,176, 100)

        self.TEXT_COLOR = (57,44,67, 255)
        self.SCROLL_BAR_COLOR = (141,102,82,255)
        
        self.BASE_WINDOW_COLOR = (29,40,54,255)
        self.INFO_WINDOW_COLOR = (37,37,38,255)
        self.ITEM_WINDOW_COLOR = (30,30,30,255)
        self.STATE_WINDOW_COLOR = (188,168,111,255)
        self.MENU_WINDOW_COLOR = (188,168,111,255)
        self.COMPONENT_WINDOW_COLOR1 = (234,222,179)
        self.COMPONENT_WINDOW_COLOR2 = (228,198,141)
        self.COMPONENT_BACKGROUND    = create_gradient_surface(self.APP_WIDTH ,self.APP_HEIGHT,  self.COMPONENT_WINDOW_COLOR1 , self.COMPONENT_WINDOW_COLOR2 )

        self.BUTTON_LIST_BG_COLOR1 = (248,240,207,205)
        self.BUTTON_LIST_BG_COLOR2 = (233,218,175)

        self.LIST_BUTTON_BG_COLOR1 = (241,211,168,205)
        self.LIST_BUTTON_BG_COLOR2 = (240,196,150,205)
        self.LIST_BUTTON_BG_COLOR  = (254,243,217, 200)

        self.COMPONENT_COLOR = (17,17,18,255)

        self.BUTTON_BG_COLOR1      = (248,219,159, 255)
        self.BUTTON_BG_COLOR2      = (248,240,207, 255)
        self.BUTTON_HOVER_COLOR    = (242,235,213, 255)
        self.BUTTON_SIDE_COLOR = (77,100,115, 255)

        self.SWITCH_BORDER_COLOR1 = (212,190,208,255)
        self.SWITCH_BORDER_COLOR2 = (142,120,138,255)

        self.SLIDER_BG_COLOR = (238,224,187,255)
        self.SLIDER_BAR_COLOR = (141,102,82,255)
        self.SLIDER_TRACK_COLOR = (243,209,91, 255)

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

        self.current_theme = 7
        self.changeTheme(self.current_theme)
    
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
    
    def updateWindowBackground(self):
        self.COMPONENT_BACKGROUND    = create_gradient_surface(self.APP_WIDTH ,self.APP_HEIGHT,  self.COMPONENT_WINDOW_COLOR1 , self.COMPONENT_WINDOW_COLOR2 )

    
    def changeTheme(self, tid):
        if tid == 1: #秋
            self.BASE_WINDOW_COLOR = (29,40,54,255)
            self.INFO_WINDOW_COLOR = (37,37,38,255)
            self.ITEM_WINDOW_COLOR = (30,30,30,255)
            self.STATE_WINDOW_COLOR = (188,168,111,255)
            self.MENU_WINDOW_COLOR = (188,168,111,255)
            self.COMPONENT_WINDOW_COLOR1 = (234,222,179)
            self.COMPONENT_WINDOW_COLOR2 = (228,198,141)

            self.BUTTON_LIST_BG_COLOR1 = (248,240,207,205)
            self.BUTTON_LIST_BG_COLOR2 = (233,218,175)

            self.LIST_BUTTON_BG_COLOR1 = (241,211,168,205)
            self.LIST_BUTTON_BG_COLOR2 = (240,196,150,205)
            self.LIST_BUTTON_BG_COLOR  = (254,243,217, 200)

            self.COMPONENT_COLOR = (17,17,18,255)

            self.BUTTON_BG_COLOR1      = (248,219,159, 255)
            self.BUTTON_BG_COLOR2      = (248,240,207, 255)
            self.BUTTON_HOVER_COLOR    = (242,235,213, 255)
            self.BUTTON_SIDE_COLOR = (77,100,115, 255)

            self.SWITCH_BORDER_COLOR1 = (212,190,208,255)
            self.SWITCH_BORDER_COLOR2 = (142,120,138,255)

            self.SLIDER_BG_COLOR = (238,224,187,255)
            self.SLIDER_BAR_COLOR = (141,102,82,255)
            self.SLIDER_TRACK_COLOR = (243,209,91, 255)

            self.TEXT_COLOR = (57,44,67, 255)
            self.updateWindowBackground()

        if tid == 2: #春
            self.BASE_WINDOW_COLOR       = (29,54,40,255)
            self.INFO_WINDOW_COLOR       = (37,38,37,255)
            self.ITEM_WINDOW_COLOR       = (30,30,30,255)
            self.STATE_WINDOW_COLOR      = (168,188,111,255)
            self.MENU_WINDOW_COLOR       = (168,188,111,255)
            self.COMPONENT_WINDOW_COLOR1 = (222,234,179)
            self.COMPONENT_WINDOW_COLOR2 = (198,228,141)

            self.BUTTON_LIST_BG_COLOR1   = (240,248,207,205)
            self.BUTTON_LIST_BG_COLOR2   = (218,233,175)

            self.LIST_BUTTON_BG_COLOR1   = (211,241,168,205)
            self.LIST_BUTTON_BG_COLOR2   = (196,240,150,205)
            self.LIST_BUTTON_BG_COLOR    = (243,254,217, 200)

            self.COMPONENT_COLOR         = (17,18,17,255)

            self.BUTTON_BG_COLOR1        = (219,248,159, 255)
            self.BUTTON_BG_COLOR2        = (240,248,207, 255)
            self.BUTTON_HOVER_COLOR      = (235,242,213, 255)
            self.BUTTON_SIDE_COLOR       = (100,115,77, 255)

            self.SWITCH_BORDER_COLOR1    = (190,212,208,255)
            self.SWITCH_BORDER_COLOR2    = (120,142,138,255)

            self.SLIDER_BG_COLOR         = (224,238,187,255)
            self.SLIDER_BAR_COLOR        = (102,141,82,255)
            self.SLIDER_TRACK_COLOR      = (209,243,91, 255)
            self.TEXT_COLOR              = (57,44,67, 255)
            self.updateWindowBackground()
        
        if tid == 3: #夏
            self.BASE_WINDOW_COLOR       = (29,174,174,255)
            self.INFO_WINDOW_COLOR       = (37,185,185,255)
            self.ITEM_WINDOW_COLOR       = (30,190,190,255)
            self.STATE_WINDOW_COLOR      = (188,222,111,255)
            self.MENU_WINDOW_COLOR       = (188,222,111,255)
            self.COMPONENT_WINDOW_COLOR1 = (179,234,222)
            self.COMPONENT_WINDOW_COLOR2 = (141,228,198)

            self.BUTTON_LIST_BG_COLOR1   = (207,248,240,205)
            self.BUTTON_LIST_BG_COLOR2   = (175,233,218)

            self.LIST_BUTTON_BG_COLOR1   = (168,241,211,205)
            self.LIST_BUTTON_BG_COLOR2   = (150,240,196,205)
            self.LIST_BUTTON_BG_COLOR    = (217,254,243, 200)

            self.COMPONENT_COLOR         = (18,17,17,255)

            self.BUTTON_BG_COLOR1        = (159,248,219, 255)
            self.BUTTON_BG_COLOR2        = (207,248,240, 255)
            self.BUTTON_HOVER_COLOR      = (213,242,235, 255)
            self.BUTTON_SIDE_COLOR       = (115,100,77, 255)

            self.SWITCH_BORDER_COLOR1    = (208,212,190,255)
            self.SWITCH_BORDER_COLOR2    = (138,142,120,255)

            self.SLIDER_BG_COLOR         = (187,238,224,255)
            self.SLIDER_BAR_COLOR        = (82,141,102,255)
            self.SLIDER_TRACK_COLOR      = (91,243,209, 255)
            self.TEXT_COLOR = (18,17,17,255)

            self.updateWindowBackground()
        
        if tid == 4: #冬
            self.BASE_WINDOW_COLOR       = (29,40,54,255)
            self.INFO_WINDOW_COLOR       = (37,37,38,255)
            self.ITEM_WINDOW_COLOR       = (30,30,30,255)
            self.STATE_WINDOW_COLOR      = (111,168,188,255)
            self.MENU_WINDOW_COLOR       = (111,168,188,255)
            self.COMPONENT_WINDOW_COLOR1 = (235,235,235,255)  # Adjusted to a lighter color
            self.COMPONENT_WINDOW_COLOR2 = (210,210,210,255)  # Adjusted to a lighter color

            self.BUTTON_LIST_BG_COLOR1   = (207,240,248,205)
            self.BUTTON_LIST_BG_COLOR2   = (175,218,233)

            self.LIST_BUTTON_BG_COLOR1   = (168,211,241,205)
            self.LIST_BUTTON_BG_COLOR2   = (150,196,240,205)
            self.LIST_BUTTON_BG_COLOR    = (217,243,254, 200)

            self.COMPONENT_COLOR         = (17,17,18,255)

            self.BUTTON_BG_COLOR1        = (159,219,248, 255)
            self.BUTTON_BG_COLOR2        = (207,240,248, 255)
            self.BUTTON_HOVER_COLOR      = (213,235,242, 255)
            self.BUTTON_SIDE_COLOR       = (77,100,115, 255)

            self.SWITCH_BORDER_COLOR1    = (208,190,212,255)
            self.SWITCH_BORDER_COLOR2    = (138,120,142,255)

            self.SLIDER_BG_COLOR         = (187,224,238,255)
            self.SLIDER_BAR_COLOR        = (82,102,141,255)
            self.SLIDER_TRACK_COLOR      = (91,209,243, 255)
            self.TEXT_COLOR              = (57,44,67,255)
            self.updateWindowBackground()

        if tid == 5: # 湛蓝之海
            self.BASE_WINDOW_COLOR       = (25,25,112,255) 
            self.INFO_WINDOW_COLOR       = (0,0,128,255) 
            self.ITEM_WINDOW_COLOR       = (0,0,139,255)  
            self.STATE_WINDOW_COLOR      = (0,0,156,255)  
            self.MENU_WINDOW_COLOR       = (0,0,205,255) 
            self.COMPONENT_WINDOW_COLOR1 = (0,0,255,255) 
            self.COMPONENT_WINDOW_COLOR2 = (70,130,180,255) 

            self.BUTTON_LIST_BG_COLOR1   = (100,149,237,205)  
            self.BUTTON_LIST_BG_COLOR2   = (135,206,235,205)  

            self.LIST_BUTTON_BG_COLOR1   = (100,149,237,205)  
            self.LIST_BUTTON_BG_COLOR2   = (240,248,255,205) 
            self.LIST_BUTTON_BG_COLOR    = (100,149,237,205)

            self.COMPONENT_COLOR         = (70,130,180,255) 

            self.BUTTON_BG_COLOR1        = (0,191,255, 255)  
            self.BUTTON_BG_COLOR2        = (30,144,255, 255)  
            self.BUTTON_HOVER_COLOR      = (70,130,180, 255)  
            self.BUTTON_SIDE_COLOR       = (25,25,112, 255) 

            self.SWITCH_BORDER_COLOR1    = (240,248,255,255)
            self.SWITCH_BORDER_COLOR2    = (240,248,255,255)  

            self.SLIDER_BG_COLOR         = (70,130,180,255)  
            self.SLIDER_BAR_COLOR        = (25,25,112,255) 
            self.SLIDER_TRACK_COLOR      = (0,191,255, 255)

            self.TEXT_COLOR              = (240,248,255,255)
            self.updateWindowBackground()
        
        
        if tid == 6: # 粉色
            self.BASE_WINDOW_COLOR       = (255,223,236,255)  
            self.INFO_WINDOW_COLOR       = (255,174,201,255)  
            self.ITEM_WINDOW_COLOR       = (255,105,180,255)  
            self.STATE_WINDOW_COLOR      = (234,153,153,255)  
            self.MENU_WINDOW_COLOR       = (255,204,229,255)  
            self.COMPONENT_WINDOW_COLOR1 = (255,174,201,255)  
            self.COMPONENT_WINDOW_COLOR2 = (255,105,180,255)  

            self.BUTTON_LIST_BG_COLOR1   = (255,223,236,205)  
            self.BUTTON_LIST_BG_COLOR2   = (255,174,201,205)  

            self.LIST_BUTTON_BG_COLOR1   = (255,223,236,205)  
            self.LIST_BUTTON_BG_COLOR2   = (255,174,201,205)  
            self.LIST_BUTTON_BG_COLOR    = (255,204,229,200)  

            self.COMPONENT_COLOR         = (255,174,201,255)  

            self.BUTTON_BG_COLOR1        = (255,105,180,255)  
            self.BUTTON_BG_COLOR2        = (234,153,153,255)  
            self.BUTTON_HOVER_COLOR      = (255,174,201,255)  
            self.BUTTON_SIDE_COLOR       = (255,223,236,255)  

            self.SWITCH_BORDER_COLOR1    = (255,223,236,255)  
            self.SWITCH_BORDER_COLOR2    = (255,174,201,255)  

            self.SLIDER_BG_COLOR         = (255,174,201,255)  
            self.SLIDER_BAR_COLOR        = (255,223,236,255)  
            self.SLIDER_TRACK_COLOR      = (255,105,180,255)  

            self.TEXT_COLOR              = (0,0,0,255) 

            self.updateWindowBackground()
        
        
        if tid == 7: #静谧夜空
            self.BASE_WINDOW_COLOR       = (0,0,50,255)     # Deeper midnight blue
            self.INFO_WINDOW_COLOR       = (0,0,40,255)     # Deeper navy
            self.ITEM_WINDOW_COLOR       = (0,0,30,255)     # Even deeper blue
            self.STATE_WINDOW_COLOR      = (0,0,20,255)     # Extremely deep blue
            self.MENU_WINDOW_COLOR       = (0,0,60,255)     # Slightly lighter deep blue

            self.COMPONENT_WINDOW_COLOR1 = (0,0,40,255)     # Deeper navy
            self.COMPONENT_WINDOW_COLOR2 = (0,0,50,255)     # Deeper midnight blue

            self.BUTTON_LIST_BG_COLOR1   = (0,0,60,205)     # Slightly lighter deep blue
            self.BUTTON_LIST_BG_COLOR2   = (0,0,30,205)     # Even deeper blue

            self.LIST_BUTTON_BG_COLOR1   = (0,0,60,205)     # Slightly lighter deep blue
            self.LIST_BUTTON_BG_COLOR2   = (0,0,30,205)     # Even deeper blue
            self.LIST_BUTTON_BG_COLOR    = (0,0,50,200)     # Deeper midnight blue

            self.COMPONENT_COLOR         = (0,0,40,255)     # Deeper navy

            self.BUTTON_BG_COLOR1        = (0,0,30,255)     # Even deeper blue
            self.BUTTON_BG_COLOR2        = (0,0,50,255)     # Deeper midnight blue
            self.BUTTON_HOVER_COLOR      = (0,0,60,255)     # Slightly lighter deep blue
            self.BUTTON_SIDE_COLOR       = (0,0,20,255)     # Extremely deep blue

            self.SWITCH_BORDER_COLOR1    = (0,0,50,255)     # Deeper midnight blue
            self.SWITCH_BORDER_COLOR2    = (0,0,40,255)     # Deeper navy

            self.SLIDER_BG_COLOR         = (0,0,40,255)     # Deeper navy
            self.SLIDER_BAR_COLOR        = (255,255,160,255)     # Slightly lighter deep blue
            self.SLIDER_TRACK_COLOR      = (55,125,55,255)     # Deeper midnight blue

            self.TEXT_COLOR              = (255,255,235,255)  # White for better contrast

            self.updateWindowBackground()

                

gol = Gol()


