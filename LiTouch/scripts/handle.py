# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import sys
import json
import os
import ast
import importlib
import subprocess
from gol import gol
from components import *

import pygame

action_funcs = {}
def scanComponentAction(type, component):
    global action_funcs
    if type == "button":
        act   = component["action"]
        action_funcs[act] = "button"

    if type == "switch":
        act   = component["action"]
        action_funcs[act] = "switch"
    
    if type == "slider":
        act   = component["action"]
        action_funcs[act] = "slider"
    
    if type == "button_list":
        list_buttons = component["buttons"]
        for list_btn in list_buttons:
            act   = list_btn["action"]
            action_funcs[act] = "list_button"

def appendComponent(type, component, module):
    if type == "label":
        label = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        color_r = component["color_r"]
        color_g = component["color_g"]
        color_b = component["color_b"]
        color_r = min(255, max(color_r, 0))
        color_g = min(255, max(color_g, 0))
        color_b = min(255, max(color_b, 0))
        gol.components.append( Label((x,y), (w,h), label, (color_r, color_g, color_b ) , cid=cid) )


    if type == "button":
        label = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        act   = component["action"]
        function = getattr(module, act) 
        gol.components.append( Button((x,y), (w,h), function, label, cid=cid) )

    if type == "switch":
        label = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        act   = component["action"]
        function = getattr(module, act) 
        gol.components.append( Switch((x,y), (w,h), function, label , cid=cid) )
    
    if type == "slider":
        label = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        act   = component["action"]
        minv  = component["min"]
        maxv  = component["max"]
        function = getattr(module, act)
        gol.components.append( Slider((x,y), (w,h), minv, maxv, function, label , cid=cid) )
    
    if type == "graph":
        label = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        gol.components.append( Graph((x,y), (w,h), label , cid=cid) )

    if type == "button_list":
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        btn_h = component["button_height"]
        list_buttons = component["buttons"]
        button_list = ButtonList((x,y),(w,h), btn_h, cid=cid)

        for list_btn in list_buttons:
            label = list_btn["label"]
            btn_cid = component["cid"]
            act   = list_btn["action"]
            function = getattr(module, act) 
            button_list.appendButton(function, label, cid=btn_cid)
        gol.components.append( button_list )
    
    if type == "mini_terminal":
        title = component["label"]
        cid   = component["cid"]
        x     = component["x"]
        y     = component["y"]
        w     = component["width"]
        h     = component["height"]
        brows = component["buffer_rows"]
        fsize = component["font_size"]
        tr    = component["text_r"]
        tg    = component["text_g"]
        tb    = component["text_b"]
        br    = component["background_r"]
        bg    = component["background_g"]
        bb    = component["background_b"]
        gol.components.append( MiniTerminal((x,y), (w,h), brows, fsize, title, (tr,tg,tb,255), (br,bg,bb,255) , cid=cid) )


module    = None
tick_func = None
init_func = None
window_title = ""


def scanAllActionFuncs():
    global module, tick_func, init_func, window_title
    if len(sys.argv) > 1:
        gol.CONFIG_PATH = sys.argv[1]

    with open(gol.CONFIG_PATH, 'r', encoding='utf-8') as f:
        data    = json.load(f)
        ui_data = data["UI"]

        gol.CALLBACK_PATH    = ui_data["action_code"]
        if (gol.CALLBACK_PATH == "config/default_action.py"):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            gol.CALLBACK_PATH = os.path.join(script_dir, "../config/default_action.py")
        window_data      = ui_data["window"]
        # scan components action
        components        = window_data["components"]
        for component in components:
            type = component["type"]
            scanComponentAction(type, component)

def readConfig():
    global module, tick_func, init_func, window_title
    if len(sys.argv) > 1:
        gol.CONFIG_PATH = sys.argv[1]
    
    print_color(f'[LiTouch] Reading config from file: {gol.CONFIG_PATH}',"green")
    with open(gol.CONFIG_PATH, 'r', encoding='utf-8') as f:
        data    = json.load(f)
        ui_data = data["UI"]

        gol.CALLBACK_PATH    = ui_data["action_code"]
        if (gol.CALLBACK_PATH == "config/default_action.py"):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            gol.CALLBACK_PATH = os.path.join(script_dir, "../config/default_action.py")
        print_color(f'[LiTouch] Action code dir : {gol.CALLBACK_PATH}',"green")

        window_data      = ui_data["window"]
        window_title     = window_data["title"]
        window_width     = window_data["width"]
        window_height    = window_data["height"]
        window_height    = max(50, window_height)


        gol.APP_WIDTH        = window_width
        gol.APP_HEIGHT       = window_height
        gol.BASE_WINDOW      = pygame.display.set_mode( (window_width, window_height) )
        gol.MENU_WINDOW      = pygame.Surface((window_width, gol.MENU_HEIGHT),  pygame.SRCALPHA) 
        gol.STATE_WINDOW     = pygame.Surface((window_width, gol.MENU_HEIGHT),  pygame.SRCALPHA)      
        gol.COMPONENT_WINDOW = pygame.Surface((window_width, window_height - 2*gol.MENU_HEIGHT),  pygame.SRCALPHA)  
          
        pygame.display.set_caption(window_title)
        gol.COMPONENT_Y   = gol.MENU_HEIGHT
        gol.STATE_Y       = gol.APP_HEIGHT - gol.MENU_HEIGHT

        # load components
        components        = window_data["components"]
        module_dir, module_file = os.path.split(gol.CALLBACK_PATH)
        module_name, _    = os.path.splitext(module_file)
        sys.path.append(module_dir)
        module = importlib.import_module(module_name)
        tick_func = getattr(module, "tick")
        init_func = getattr(module, "init")
        for component in components:
            type = component["type"]
            appendComponent(type, component, module)


        print_color("read config successfully","green")

def checkAndReadConfig():
    global action_funcs
    action_funcs = {}
    action_funcs["init"] = "button"
    action_funcs["tick"] = "tick"
    scanAllActionFuncs()

    required_funcs = {}
    params = []
    for key,value in action_funcs.items():
        if value == "button" or value == "list_button":
            params = []
        if value == "switch":
            params = ['state']
        if value == "slider":
            params = ['value']
        if value == "tick":
            params = ['dt']
        required_funcs[key] = params


    try:
        with open(gol.CALLBACK_PATH) as f:
            content = f.read()
        
        tree               = ast.parse(content)
        existing_functions = {}
        for node in ast.walk(tree):
            if isinstance(node,ast.FunctionDef):
                existing_functions[node.name] = [arg.arg for arg in node.args.args]
        missing_functions = {}

        for func_name, params in required_funcs.items():
            if func_name not in existing_functions:
                missing_functions[func_name] = params
            elif existing_functions[func_name] != params:
                missing_functions[func_name] = params

        with open(gol.CALLBACK_PATH, 'a') as f:
            for func, params in missing_functions.items():
                print_color("[LiTouch] 缺少回调函数："+func+" , 将会自动补全", "yellow")
                f.write(f"\ndef {func}({', '.join(params)}):\n pass\n")

        missing_func_count = len(missing_functions.items())
        if missing_func_count > 0:
            print_color("[Litouch] 发现" + str(missing_func_count) + "个缺失的回调函数，已补全， 请打开action code编写回调功能", "yellow")
 
        
        readConfig()

    except Exception as e:
        print_color("[Litouch] Error occurred : " + type(e).__name__ , "red")

def saveConfig():
    components_data = []

    for component in gol.components:
        component_dict = {}

        if isinstance(component, Label):
            component_dict["type"] = "label"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.content
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            r,g,b = component.color
            component_dict["color_r"] = r
            component_dict["color_g"] = g
            component_dict["color_b"] = b

        elif isinstance(component, Button):
            component_dict["type"] = "button"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.title
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            component_dict["action"] = component.callback.__name__

        elif isinstance(component, Switch):
            component_dict["type"] = "switch"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.title
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            component_dict["action"] = component.callback.__name__

        elif isinstance(component, Slider):
            component_dict["type"] = "slider"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.title
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            component_dict["min"] = component.min_value
            component_dict["max"] = component.max_value
            component_dict["action"] = component.callback.__name__

        elif isinstance(component, Graph):
            component_dict["type"] = "graph"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.title
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
        
        elif isinstance(component, ButtonList):
            component_dict["type"] = "button_list"
            component_dict["cid"]  = component.cid
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            component_dict["button_height"] = component.button_height

            buttons_data = []
            for btn in component.button_list:
                btn_dict = {}
                btn_dict["label"]  = btn.title
                btn_dict["cid"]    = btn.cid
                btn_dict["action"] = btn.callback.__name__
                buttons_data.append(btn_dict)
            
            component_dict["buttons"] = buttons_data
        
        elif isinstance(component, MiniTerminal):
            component_dict["type"] = "mini_terminal"
            component_dict["cid"]  = component.cid
            component_dict["label"] = component.title
            component_dict["x"], component_dict["y"] = component.coord
            component_dict["width"], component_dict["height"] = component.size
            component_dict["buffer_rows"] = component.buffer_rows
            component_dict["font_size"] = component.font_size
            r,g,b,_ = component.text_color
            component_dict["text_r"] = r
            component_dict["text_g"] = g
            component_dict["text_b"] = b
            r,g,b,_ = component.bg_color
            component_dict["background_r"] = r
            component_dict["background_g"] = g
            component_dict["background_b"] = b

        components_data.append(component_dict)

    ui_data = {
        "action_code": gol.CALLBACK_PATH,
        "window": {
            "title": pygame.display.get_caption()[0],
            "width": gol.APP_WIDTH,
            "height": gol.APP_HEIGHT,
            "components": components_data
        }
    }

    data = {
        "UI": ui_data
    }

    with open(gol.CONFIG_PATH, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=4)

    print_color("[LiTouch] Configuration saved successfully.", "green")

def configOpen():
    try:
        # 打开指定文件或目录在VSCode中
        subprocess.run(["code", gol.CONFIG_PATH])
    except:
        try:
            # 如果VSCode打不开，则尝试用gedit打开
            subprocess.run(["gedit", gol.CONFIG_PATH])
        except Exception as e:
            print_color(f"An error occurred while trying to open with gedit: {e}", "red")

def templateOpen():
    try:
        # 打开指定文件或目录在VSCode中
        subprocess.run(["code", gol.TEMPLATE_CONFIG_PATH])
        subprocess.run(["code", gol.TEMPLATE_CALLBACK_PATH])
    except:
        try:
            # 如果VSCode打不开，则尝试用gedit打开
            subprocess.run(["gedit", gol.TEMPLATE_CONFIG_PATH])
            subprocess.run(["gedit", gol.TEMPLATE_CALLBACK_PATH])
        except Exception as e:
            print_color(f"An error occurred while trying to open with gedit: {e}","red")



def callbackCodeOpen():
    try:
        # 打开指定文件或目录在VSCode中
        subprocess.run(["code", gol.CALLBACK_PATH])
    except:
        try:
            # 如果VSCode打不开，则尝试用gedit打开
            subprocess.run(["gedit", gol.CALLBACK_PATH])
        except Exception as e:
            print_color(f"An error occurred while trying to open with gedit: {e}","red")


def layoutCallback( state ):
    if state == False:
        print_color("[LiTouch] 关闭调整布局","yellow")
        for component in gol.components:
            component.OnModify = False
    if state == True:
        print_color("[LiTouch] 开启调整布局","yellow")
        for component in gol.components:
            component.OnModify = True


def topConfig(state ):
    global window_title
    cmd_str = 'wmctrl -l | grep \"' + window_title + '\"'
    window_id = subprocess.getoutput(cmd_str)
    if state == True:
        subprocess.call(['wmctrl','-i','-r', window_id, '-b', 'add,above' ])
        print_color("[Litouch] 已置顶","green")
    else:
        subprocess.call(['wmctrl','-i','-r', window_id, '-b', 'remove,above' ])
        print_color("[Litouch] 取消置顶","green")



def init():
    global init_func
    w  = gol.APP_WIDTH
    mw = w / 8
    iw = mw * 0.9
    tw = mw * 0.05
    left_3 = [(8+tw + mw * i,8) for i in range(3)]
    m1_str = "Layout"
    m2_str = "save"
    m3_str = "Top"
    if(iw < 50):
       m1_str = "L"
       m2_str = "S" 
       m3_str = "T"
    m1 = Switch( left_3[0],   (iw, gol.MENU_HEIGHT - 16), layoutCallback, m1_str)
    m2 = Button( left_3[1],   (iw, gol.MENU_HEIGHT - 16), saveConfig,     m2_str)
    m3 = Switch( left_3[2],   (iw, gol.MENU_HEIGHT - 16), topConfig,      m3_str)

    right_4 = [ (gol.APP_WIDTH - tw - mw*(i+1) , 8) for i in range(4)]
    m4_str = "Action code"
    m5_str = "Config code"
    m6_str = "Template"
    m7_str = "Gene Code"
    if(iw < 50):
       m4_str = "A"
       m5_str = "C" 
       m6_str = "M"
       m7_str = "G"
    elif(iw < 80):
       m4_str = "Action"
       m5_str = "Config" 
       m6_str = "Tem"
       m7_str = "Gene"
    m4 = Button( right_4[0],   (iw, gol.MENU_HEIGHT - 16), callbackCodeOpen, m4_str )
    m5 = Button( right_4[1],   (iw, gol.MENU_HEIGHT - 16), configOpen, m5_str )
    m6 = Button( right_4[2],   (iw, gol.MENU_HEIGHT - 16), templateOpen, m6_str )

    gol.menu.append(m1)
    gol.menu.append(m2)
    gol.menu.append(m3)
    gol.menu.append(m4)
    gol.menu.append(m5)
    gol.menu.append(m6)

    s1 = StateBar( (0,0), (gol.APP_WIDTH, gol.MENU_HEIGHT) )
    gol.state_bar = s1
    gol.states.append(s1)

    # test here
    # g = Graph((300,700),(300,200), "graph")
    # gol.components.append(g)

    # g.createCurve("c1", gol.COLOR_ORANGE)
    # for i in range(100):
    #     g.appendData("c1", i)
    
    init_func()


def tick(dt):
    global tick_func
    tick_func(dt)
