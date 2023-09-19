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

# 在此处添加自定义回调函数
def button1_callback():
    print("Slider演示")
    interface.pubLiTouchCmd(6)

def button2_callback():
    print("清除所有已发布内容")
    interface.pubLiTouchCmd(7)
    interface.launchLiTouch()

def slider_callback( value ):
    interface.pubLiTouchCmd(8, [value] )

def demo1_callback():
    print("播放demo1")
    interface.pubLiTouchCmd(1)
    
def demo2_callback():
    print("播放demo2")
    interface.pubLiTouchCmd(2)

def demo3_callback():
    print("播放demo3")
    interface.pubLiTouchCmd(3)

def demo4_callback():
    print("播放demo4")
    interface.pubLiTouchCmd(4)

def demo5_callback():
    print("播放demo5")
    interface.pubLiTouchCmd(5)

