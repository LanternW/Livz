# Copyright (c) [2023] [Lantern]
# 
# This file is part of [Livz]
# 
# This project is licensed under the MIT License.
# See LICENSE.txt for details.
import os
import subprocess
from gol import *
from ros_interface import pubLiTouchCmd
from components import getComponentByName, getComponentByCid


install_prefix = os.environ.get('LIVZ_TOUCH_LIB_VAR', './')


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

def printToTerminal(name, content):
    terminal = getComponentByName(name)
    if terminal != None:
        terminal.feedString(content)


def launchLiTouch(config_path = install_prefix + "/../config/default.json"):
    python_command = "python"
    versions = ["3.10", "3.9", "3.8", "3.7", "3.6", "3.5", "3.4", ""]

    for version in versions:
        command = ["python" + version, "--version"]
        try:
            subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            python_command += version
            print_color(f"[LiTouch] 使用{python_command}启动。", "green")
            break
        except Exception as e:
            continue

    # 创建并启动子进程
    LIVZ_TOUCH_PATH = install_prefix + "/main.py"
    try:
        args = [python_command, LIVZ_TOUCH_PATH]
        if config_path:
            args.append(config_path)

        subprocess.Popen(args)

    except Exception as e:
        print_color(f"Fork failed: {e}" , red)

