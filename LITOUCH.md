### LiTouch 定制化交互界面
---
LiTouch是一个基于pygame的图形化定制UI工具，通过集成ROS1来实现与C++程序的通信。同时，在Livz可视化工具中提供了一些接口便于和LiTouch交互。

### 安装依赖（除了ROS环境外的）
```bash
pip intsll pygame
```
注意：python3.11与ROS环境存在一些兼容性问题，无法新建rosnode。因此，你的电脑上最好有3.10及以下的python版本。Livz的启动API将会自动寻找合适的版本。

### 运行example示例程序
---
```bash
cd Livz/example
catkin_make
source devel/setup.bash
roslaunch example run_demo.launch
```
这个演示程序提供了一些使用Livz可视化的示例，同时，使用LiTouch工具来提供交互操作。

这个演示程序的流程如下：

首先，在main.cpp中，调用API来启动LiTouch，并传递UI配置文件。

```C++
Livz::launchLiTouch("ui_config_path");
```
UI配置文件的地址为
```
example/src/example/config/touch_layout.json
```
程序中使用了绝对路径，因此，如果LiTouch没有正确启动，请首先修改UI配置文件的路径。如果你使用VSCode,可以在资源管理器的文件上右键->复制路径。

然后，调用API绑定Action回调函数
```C++
Livz::setLitouchAction(litouchAction);
```
Action回调函数的类型必须为：
```C++
[](const int cmd_id, const std::vector<double> params)->void
```
由于LiTouch是 python实现的，而用户程序可能是C++，因此在Livz内部定义了一个接口用于将Litouch的组件交互信息传递到C++中。

实现组件的交互回调需要编写回调逻辑，LiTouch的回调逻辑统一编写在一个.py文件中，这个文件的路径由UI配置文件中"action_code"属性指定。同样的，示例程序中使用了绝对路径，因此需要进行修改。示例程序使用的回调逻辑文件目录为：
```bash
example/src/example/config/touch_action.py
```
在回调逻辑文件中，定义了每一个组件的动作响应。其中
```python
pubLiTouchCmd(cmd_id, [params] )
```
是interface提供的API，它负责发送一个LiTouchCmd消息，可以附带任意长度的浮点数参数，这个消息会被Action回调函数所接收，从而实现python程序与C++的通信。



### UI配置文件和回调逻辑文件
---
如示例程序所介绍，自定义一个LiTouch UI需要两个文件：UI配置文件和回调逻辑文件。其中UI配置文件的格式为 .json, 回调逻辑文件的格式是 .py

UI配置文件的构成如下（注意，json文件中不支持注释，实际使用时务必不要带“//”注释）：
```json
{
    "UI": {
        "action_code": "/path/to/your/action.py", //回调逻辑文件路径
        "window": {
            "title": "LiTouch", //窗口标题
            "width": 800,       //窗口尺寸
            "height": 600,
            "components": [      //组件列表
                {
                    "type": "label",
                    "cid": -1,
                    "label": "Demos",
                    "x": 2,
                    "y": 38,
                    "width": 120,
                    "height": 36,
                    "color_r": 255,
                    "color_g": 255,
                    "color_b": 120
                },
                ......
```
而回调逻辑文件的构成如下：
```python
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
......
```
如果想要快速编写这两个文件，可以如下操作：首先使用不带参数的API启动LiTouch:
```C++
Livz::launchLiTouch();
```
这会打开默认配置，然后点击菜单栏上的 [Template] 按键，这将打开UI配置文件和回调逻辑文件的模板，可以按照模板内容修改。此外，你可以着重于修改UI，而无需关注回调逻辑文件，因为每次启动时，将自动补全缺少的回调函数，无需手动编写回调逻辑文件框架。

### 菜单栏和底部状态栏功能
---

LiTouch上方的菜单栏不能修改。它包含6个功能键。注意：当窗口尺寸太小时，菜单按钮会变成缩写。

-- Layout  开启/关闭 布局模式，开启布局模式后可以对窗口中的组件进行移动和缩放。

-- Save 保存当前布局,这将覆盖UI配置文件的内容

-- Top 开启/关闭 窗口置顶

-- Template 打开配置文件的模板

-- Config code 打开 UI配置文件

-- Action code 打开回调逻辑文件

底部状态栏默认会记录最近一次操作，同时可以进行闪烁提示。LiTouch也提供API控制底部状态栏进行不同类型的闪烁。


### 组件
---

目前，LiTouch包含的组件为
1. 静态文字 "label"
2. 按钮 "button"
3. 开关 "switch"
4. 按钮列表 "button_list" 
5. 滑块 "slider"
6. 迷你终端 "mini_terminal"


#### 静态文字 Label
静态文字组件用于在LiTouch中显示静态标题，没有交互行为。但是，你可以在回调逻辑中修改它的内容，将它作为状态监视器使用。静态文字组件的属性如下：
```json
{
    "type": "label",
    "cid": -1,               //标识符，配合getComponentByCid使用
    "label": "static label", //标题内容
    "x": 259,                //x,y,width,height是位置和尺寸
    "y": 20,
    "width": 496,
    "height": 47,
    "color_r": 255,          //颜色
    "color_g": 120,
    "color_b": 120
}
```
"type","cid","x","y","width","height"这些参数大部分组件都有，后面不再赘述。
静态文字的字体大小是根据尺寸和文字内容自动调节的。

#### 按钮 Button
按钮组件提供单击的交互行为。它的回调函数没有参数。按钮的属性如下：
```json
{
    "type": "button",
    "cid": -1,
    "label": "Button1",       //标题内容
    "x": 276,
    "y": 358,
    "width": 471,
    "height": 47,
    "action": "button1_callback"   //回调函数名
}
```

#### 开关 Switch
开关组件提供布尔状态改变的交互行为。它的回调函数有一个参数，表示当前开关状态。开关的属性如下：
```json
{
    "type": "switch",
    "cid": -1,
    "label": "Switch1",
    "x": 640,
    "y": 441,
    "width": 30,
    "height": 30,
    "action": "switch_callback"
}

```
开关在关闭状态下，左右两侧有橙色竖线，开启状态下是被橙色框包围。

#### 按钮列表 ButtonList
需要大量按钮的场景，可以使用按钮列表组件。它包括若干个列表按钮，每一个列表按钮都需要定义一个没有参数的回调函数。按钮列表的属性如下：
```json
{
  "type": "button_list",
  "cid": -1,
  "x": 27,
  "y": 79,
  "width": 228,
  "height": 419,
  "button_height": 40,   //列表中每个按钮的高度
  "buttons": [           //每一个列表按钮只需要3个属性
      {
          "label": "run_demo1",    //标题
          "cid": -1,               //标识符
          "action": "demo1_callback"  //回调函数
      },
      {
          "label": "run_demo2",
          "cid": -1,
          "action": "demo2_callback"
      },
      ......
  ]
}
```

#### 滑动条 Slider
滑动条组件提供连续浮点数变动的交互行为。它的回调函数有一个参数，表示当前滑块的值。滑动条的属性如下：
```json
{
    "type": "slider",
    "cid": -1,
    "label": "Slider",
    "x": 260,
    "y": 87,
    "width": 496,
    "height": 46,
    "min": 0,                       // 滑块值的区间
    "max": 100,
    "action": "slider_callback"
}
```

#### 迷你终端 MiniTerminal
迷你终端提供简单的终端模拟功能，它可以用作监视器接收日志输出，也可以实现简单的命令交互（尽管现在没有多少命令可以用）。你可以在LiTouch中放置多个迷你终端，然后用Livz提供的API向不同的终端发送不同的信息，这比所有信息都输出到系统终端更整洁。迷你终端的属性如下：
```json

{
    "type": "mini_terminal",
    "cid": -1,
    "label": "terminal1",
    "x": 280,
    "y": 83,
    "width": 142,
    "height": 158,
    "buffer_rows": 200,           //缓存区容量（回滚行数）
    "font_size": 14,              //字体大小
    "text_r": 30,                 //字体颜色
    "text_g": 255,
    "text_b": 30,
    "background_r": 0,            //背景颜色
    "background_g": 0,
    "background_b": 0
}
```
迷你终端提供指令交互功能。单击一个迷你终端，出现黄色光标闪烁表示进入编辑状态，再次单击会退出编辑状态。因此，你可以单击多个迷你终端实现指令同步。

和其它终端类似，支持方向键唤出历史记录、Tab补全功能、Ctrl+C取消功能。

目前支持的指令如下：

-- clear 清除终端缓存区

-- ip 查看本机所有IPV4地址

-- exportToFile [file_name] 将终端缓存区内容导出到文件，文件放在回调逻辑函数同一级目录

-- setTextColor [r] [g] [b] 设置字体颜色

-- setBgColor [r] [g] [b] 设置背景颜色


### 接口和API
---
在回调逻辑文件的模板中，引入interface来为自定义内容提供一些API。

LiTouch端的API包括：

-- pubLiTouchCmd(id, params = []) 发布LiTouchCmd消息，可以被Livz绑定的回调函数接收

-- flashInfo(info) 底部状态栏闪烁

-- flashWarning(info) 底部状态栏闪烁，背景为橙色（警告）

-- flashError(info) 底部状态栏闪烁，背景为红色（错误）

-- flashGood(info)  底部状态栏闪烁，背景为绿色（喜报）

-- getComponentbyName(name) 通过名称获取组件

-- getComponentvbyCid(cid) 通过cid获取组件，假设你想要让一个Label的内容经常变动，那么通过名称获取组件显然不是一个明知的方法。

除了LiTouch端提供的API，Livz也提供了一些API来负责C++部分与LiTouch的交互，这些API包括：

-- Livz::launchLiTouch(const string& UI_conf_path); 使用指定配置文件启动LiTouch，如果不传递参数，则使用默认配置。会自动寻找合适的python版本启动LiTouch

-- Livz::setLitouchAction( [](const int cmd_id, const std::vector<double> params)->void f); 绑定LiTouchCmd回调函数。

-- Livz::LTout(const string& terminal_name) 创建一个向LiTouch中名称为terminal_name的迷你终端的“输出流”。使用方法与cout类似，但是刷新缓冲标志要使用LTEndl。
