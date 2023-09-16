import os
import sys
install_prefix = os.environ.get('LIVZ_TOUCH_LIB_VAR', './')
sys.path.append(install_prefix)
import interface
################################################################
################################################################

# 必须包含一个init函数，该函数在初始化时被调用一次，用于创建自定义内容
def init():
    pass

# 必须包含一个tick函数，这个函数将在每一帧被调用，dt为距离上一次调用经过的时间（毫秒）
def tick(dt):
    pass

################################################################
################################################################

# 在下面添加自定义回调函数
def button1_callback():
    print("button1 click")

def button2_callback():
    print("button2 click")

def listbtn1_callback():
    print("list button1 click")
    
def listbtn2_callback():
    print("list button2 click")

def listbtn3_callback():
    print("list button3 click")

def slider_callback( value ):
    print("slider value : " , value)


