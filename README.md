# The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense
an intelligent vehicle system for real-time laser combat with autonomous navigation, target recognition, tracking, and precision laser striking.
# The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense

an intelligent vehicle system for real-time laser combat with autonomous navigation, target recognition, tracking, and precision laser striking.

# 目录

1. 摘要
2. 硬件系统组成
3. 代码说明

# 摘要

**本设计为实现实时激光对抗的智能车系统，目的是设计一款集成人工智能视觉处理等先进技术的精准避障、实时识别追踪的光电智能小车，使其具备自主寻迹避障、目标识别、目标追踪打击等功能，通过实时路径规划提高目标寻找和打击效率，实时检测敌方小车与打击区位置，实现高精度目标打击。**

**系统由智能车部分和视觉装置部分组成。智能车部分包含主控模块、环境感知模块、驱动模块和电源模块。该部分主要运用了四元数算法，并结合磁场椭球拟合，对磁力计数据进行实时椭球校准，降低硬/软磁干扰，减少了偏航误差。驱动模块搭载串级PID算法，实现高精度运动控制。利用卡尔曼滤波，互补滤波等算法，改善数据平滑度，滤除噪声，提升数据解算的准确度。**

**视觉装置部分由视觉识别部分与目标追踪打击部分构成。视觉识别部分将树莓派与Hailo8 NPU高性能张量计算单元结合，搭载yolov11n模型实现稳定90帧识别定位打击区贴纸；目标追踪打击部分由二维云台搭载优化后的PID算法实现40hz刷新率的精确打击**

**整个系统采用多传感器融合、超声波测距、惯性导航运动计算、视觉硬件加速等方法，通过硬件加速、数据融合、算法优化等手段，得出了系统在动态目标识别、精准测距与打击等方面的良好结果，实现了高精准辨识、自动行驶和智能打击等功能，能准确避开障碍、正确快速识别目标打击对象并高效完成打击任务。**

**系统的创新点为采用高性能hailo8 NPU 搭载先进yolov11n模型实现工业级高帧率图像解算，以高帧率高实时性解决高速运动模糊问题、优化 PID 算法改善系统响应、运用九轴姿态解算的四元数算法组合减少偏航误差**

# 硬件组成

**(一)智能车部分**

1.主控模块MCU：该模块采用 STM32 单片机，其作为光电智能小车的核心控制中心，负责管理和协调其余模块的正常运行，各单片机间通过串口通信连接。

2.环境感知模块：该模块包含了测距模块，姿态与导航模块。测距模块采用超声波传感器，通过发送超声波并接受返回超声波测得小车位置；姿态与导航模块采用ICM20948，采集加速度、角速度、磁场数据，解算车体姿态角，计算运动轨迹。两模块所得数据互相矫正，共同作用得到更为准确的小车位置信息。

3.驱动模块：TB6612电机驱动模块

4.电源模块：采用 12V锂电池电源，为各模块提供稳定的电力支持，以确保系统正常运行。

**（二）视觉装置部分**

该部分由视觉识别部分与目标追踪打击部分构成。视觉识别部分主要由树莓派5，Hailo8 NPU和摄像头组成，搭载 YOLOv11n目标检测模型实现中场信号灯、敌对小车与打击区的识别与定位，其中模型推理和图像后处理由Hailo8 NPU实现硬件加速，保证高帧率、高实时性；目标追踪打击部分主要由二维云台与小功率激光模块组成，依据实时检测数据配合优化后的PID算法实现目标追踪、激光打击。视觉装置和智能车通过串口通信实现信息和状态的共享。

# 代码说明

## 树莓派5部分

### Dependence：

1. 系统环境配置：树莓派5使用官方系统，详见[官网](https://www.raspberrypi.com/software/)
2. 树莓派配置hailo-8 NPU环境`sudo apt install hailo-all`，详见[官网](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html#ai-hat-plus)
3. 配置python项目环境，详见[项目](https://github.com/hailo-ai/hailo-rpi5-examples)

### QuickStart

bash

`source ./venv_hailo_rpi5_examples/bin/activate # 激活虚拟环境`

`python  ./Raspi-code/mian.py #启动程序`

### 代码结构
-Raspi-code
-----------[utils.py](https://github.com/BuPeiChiTang/The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense/blob/main/Raspi-code/utils.py "utils.py")#必要组件1

-----------[object\_detection\_utils\_final.py](https://github.com/BuPeiChiTang/The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense/blob/main/Raspi-code/object_detection_utils_final.py "object_detection_utils_final.py")#必要组件2

-----------[mian.py](https://github.com/BuPeiChiTang/The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense/blob/main/Raspi-code/mian.py "mian.py")

-----------[yolov11n.hef](https://github.com/BuPeiChiTang/The-intelligent-vehicle-equipped-with-the-Hailo-8-NPU-for-laser-defense/blob/main/Raspi-code/yolov11n.hef "yolov11n.hef")#我们使用的目标识别模型

-----------else

