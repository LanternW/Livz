#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import math
import pygame
import time
from gol import *
import random
import numpy as np

def getRandomColor():
    r = random.randint(50, 255)
    g = random.randint(50, 255)
    b = random.randint(50, 255)
    return (r, g, b)

def get_contrast_color(rgb_color):
    # 计算反色
    r,g,b,a = rgb_color
    contrast_color = (255-r, 255-g, 255-b, a)
    return contrast_color


def colorGradient(color1, color2, scale):
    nr = scale*color2[0] + (1-scale)*color1[0]
    ng = scale*color2[1] + (1-scale)*color1[1]
    nb = scale*color2[2] + (1-scale)*color1[2]
    na = scale*color2[3] + (1-scale)*color1[3]
    return (nr,ng,nb,na)

def limitValue( value, min, max):
    if value >= max:
        value = max
    if value <= min:
        value = min
    return value

def float_to_str(float_num, max_round = 2):
    rounded_num = round(float_num, max_round)
    rounded_num *= (10 ** max_round) 
    rounded_num /= (10 ** max_round) 
    str_num = str(rounded_num)
    return str_num

def dis2D(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.sqrt( dx*dx + dy*dy)

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def blitTextCenter( surface, text, font, text_center, color ):
    text_obj  = font.render(text, True, color)
    text_rect = text_obj.get_rect(center = text_center)
    surface.blit( text_obj, text_rect )
    return text_rect

def blitTextLeft(surface, text, font, leftmid_coord, color):
    text_obj = font.render(text, True, color)
    text_rect = text_obj.get_rect()
    text_rect.midleft = leftmid_coord
    surface.blit(text_obj, text_rect)

def blitTextRight(surface, text, font, rightmid_coord, color):
    text_obj = font.render(text, True, color)
    text_rect = text_obj.get_rect()
    text_rect.midright = rightmid_coord
    surface.blit(text_obj, text_rect)

def blitTextLeftWithBG( surface, text, font, leftup_coord , color , bg_color):
    text_obj  = font.render(text, True, color)
    text_rect = text_obj.get_rect()
    rect_coord = text_rect.topleft
    rect_coord = (rect_coord[0] + leftup_coord[0], rect_coord[1] + leftup_coord[1])
    rect_size  = text_rect.size
    pygame.draw.rect( surface, bg_color, (rect_coord, rect_size), 0)
    surface.blit( text_obj, leftup_coord )

def renderCornerRect( screen, color, coord, size , width):
    pygame.draw.rect( screen, color, (coord, size), width)

def renderCircle( screen, color, coord, radius , width):
    pygame.draw.circle( screen, color, coord, radius, width)


def renderLine( surface, color, start , end, width):
    pygame.draw.line( surface, color, start, end ,width)

def renderDottedLine( surface, color, start , end, width, gap):
    dis = dis2D( start, end )
    num = math.floor( dis/gap )
    if num == 0:
        num = 1
    dt = 1.0 / num
    for i in range(0, num):
        if i%2 == 0:
            s1 = dt * i
            s2 = dt * (i+1)
            p1 = ( s1*end[0] + (1-s1)*start[0] ,  s1*end[1] + (1-s1)*start[1] )
            p2 = ( s2*end[0] + (1-s2)*start[0] ,  s2*end[1] + (1-s2)*start[1] )
            renderLine( surface, color, p1, p2, width )

def isMouseInWindow( mouse_x, mouse_y, window_pos, window_size):
    relative_x  = mouse_x - window_pos[0]
    relative_y  = mouse_y - window_pos[1]
    return isPointInWindow( (relative_x, relative_y) , window_size )

def isPointInWindow( point_coord, window_size):
    if point_coord[0] >= 0 and point_coord[0] <= window_size[0] and point_coord[1] >= 0 and point_coord[1] <= window_size[1]:
        return True
    else:
        return False

def isPointInWindowEx( point_coord, window_size, expand_dis):
    point2_coord = (point_coord[0] + expand_dis,  point_coord[1] + expand_dis)
    if expand_dis > 0:
        if isPointInWindow( point_coord, window_size ) == True or isPointInWindow( point2_coord, window_size ) == True:
            return True
        else:
            return False
    else:
        if isPointInWindow( point_coord, window_size ) == True and isPointInWindow( point2_coord, window_size ) == True:
            return True
        else:
            return False


def canItemStackUp(item1, item2):
    return False 
    if item1.uuid != "None" or item2.uuid != "None":
        return False
    
    if item1.is_group == True or item2.is_group == True:
        return False
    
    a = ( item1.name == item2.name )
    b = ( item1.type == item2.type )
    c = ( item1.group == item2.group )
    d = ( item1.state == item2.state )
    
    if a and b and c and d:
        return True
    return False

def sigmoid(x):
    # x:[0,1] -> y:[0,1]
    x = (x - 0.5) * 10
    return 1/(1 + math.exp(-x))

def getDateStr():
    return time.strftime('%Y-%m-%d %T',time.localtime(time.time()))


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

def create_gradient_surface_with_radius(width_, height_, start_color, end_color, radius = 0.2, dir="vertical"):
    width  = int(height_)
    height = int(width_)
    if dir == 'horizontal':
        gradient = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            color = (
                start_color[0] * (1 - abs(2*y/height - 1)) + end_color[0] * abs(2*y/height - 1),
                start_color[1] * (1 - abs(2*y/height - 1)) + end_color[1] * abs(2*y/height - 1),
                start_color[2] * (1 - abs(2*y/height - 1)) + end_color[2] * abs(2*y/height - 1)
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
    
    # 创建圆角
    surface = pygame.surfarray.make_surface(gradient)
    surface = surface.convert_alpha()
    mask    = pygame.Surface((width_, height_), pygame.SRCALPHA)
    pygame.draw.rect(mask, (255, 255, 255), mask.get_rect(), 0, border_radius=int(min(width, height)*radius/2))
    surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

    return surface

def create_soft_shadow(width, height, radius=0.5, blur=10):

    width = int(width)
    height = int(height)
    abs_radius = int(min(width, height)*radius/2)
    def getSDF(x,y, dx,dy):
        sdf = 0
        if dx < abs_radius and dy < abs_radius:
            rmin = math.sqrt( (x-abs_radius)*(x-abs_radius) + (y-abs_radius)*(y-abs_radius) )                   
            rmin = min(rmin, math.sqrt( (x - width + abs_radius)*(x-width + abs_radius) + (y-abs_radius)*(y-abs_radius)) )
            rmin = min(rmin, math.sqrt( (x-abs_radius)*(x-abs_radius) + (y-height+ abs_radius)*(y-height+ abs_radius)))
            rmin = min(rmin, math.sqrt( (x- width+abs_radius)*(x- width+abs_radius) + (y-height+ abs_radius)*(y-height+ abs_radius)))
            sdf  = rmin - abs_radius
        else:
            kmin = min(dx, dy)
            sdf  = -kmin
        return sdf


    surface = pygame.Surface((width, height), pygame.SRCALPHA)
    # surface.fill((40,40,40,50))
    for y in range(height):
        for x in range(width):
            dx = min(x, width - x)
            dy = min(y , height - y)
            kmin = min(dx, dy)
            if (kmin > 20):
                continue

            sdf   = getSDF(x,y, dx, dy)
            alpha = int( min(255, max(0, (-255 * sdf / blur) )))
            color = (161,131,125,alpha)
            surface.set_at((x, y), color)

    mask = pygame.Surface((width, height), pygame.SRCALPHA)
    pygame.draw.rect(mask, (255, 255, 255), mask.get_rect(), 0, border_radius=abs_radius)
    surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

    return surface