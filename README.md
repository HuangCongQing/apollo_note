# Apollo
百度的自动驾驶平台Apollo项目-学习

* 官方：https://apollo.auto/devcenter/idpcourse_cn.html
* github：https://github.com/ApolloAuto/apollo


**Apollo是百度的自动驾驶开源框架，根据自动驾驶的功能划分为不同的模块**

主要分为6部分：

- **高精度地图 HD Maps、**
- **定位 Localization**
- **感知 Perception**
- **预测 Prediction**
- **规划 Planning**
- **控制 Control**

- 1：高精度地图 HD Maps（High Definition Maps）
- 2：定位 Localization： 汽车定位自己位置厘米级精度自定位
- 3：感知 Perception：深度学习（CNN 分类 检测 分割）数据来源：摄像头 雷达 激光雷达
- 4：预测 Prediction： 预测其他车辆或行人可能如何移动（RNN）
- 5：规划 Planning： 将预测和路线相结合以生成车辆轨迹
- 6：控制 Control： 如何使用转向，油门，制动（刹车）来执行规划轨迹


## Apollo各部分架构

<br />

- 定位
   - [https://github.com/ApolloAuto/apollo/blob/master/modules/localization/README_cn.md](https://github.com/ApolloAuto/apollo/blob/master/modules/localization/README_cn.md)
   - 一种是结合GPS和IMU信息的RTK（Real Time Kinematic实时运动）方法，另一种是融合GPS、IMU和激光雷达信息的多传感器融合方法。
- 感知
   - [https://github.com/ApolloAuto/apollo/tree/master/modules/perception](https://github.com/ApolloAuto/apollo/tree/master/modules/perception)
- 预测
   - [https://github.com/ApolloAuto/apollo/blob/master/modules/prediction/README_cn.md](https://github.com/ApolloAuto/apollo/blob/master/modules/prediction/README_cn.md)
   - 预测模块从感知模块接收障碍物，其基本感知信息包括位置、方向、速度、加速度，并生成不同概率的预测轨迹。
- 路由
   - [https://github.com/ApolloAuto/apollo/blob/master/modules/routing/README_cn.md](https://github.com/ApolloAuto/apollo/blob/master/modules/routing/README_cn.md)
   - 路由模块根据请求生成高级导航信息。

路由模块依赖于路由拓扑文件，通常称为Apollo中的routing_map.*。路由地图可以通过命令来生成。

- 规划
   - [https://github.com/ApolloAuto/apollo/blob/master/modules/planning/README_cn.md](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/README_cn.md)
   - <br />
- 控制
   - [https://github.com/ApolloAuto/apollo/blob/master/modules/control/README_cn.md](https://github.com/ApolloAuto/apollo/blob/master/modules/control/README_cn.md)
   - 本模块基于规划和当前的汽车状态，使用不同的控制算法来生成舒适的驾驶体验。控制模块可以在正常模式和导航模式下工作。
   - 输出：**给底盘的控制指令（转向，节流，刹车）。**

**<br />**
<a name="QAQU5"></a>
## 无人驾驶车运作方式(Apollo 平台的无人驾驶结构)
Apollo 无人驾驶平台是以**高精地图和定位模块**作为核心。其他的模块都是以这两个模块为基础。我们会在整个课程中看到这样的结构。<br />
<br />![](https://cdn.nlark.com/yuque/0/2020/png/232596/1599535320908-241a7daa-7d99-49f6-af06-dd15aaf0c1ad.png#align=left&display=inline&height=405&margin=%5Bobject%20Object%5D&originHeight=764&originWidth=1153&size=0&status=done&style=none&width=611)<br />

<a name="hb75R"></a>
## Apollo团队和架构
**pollo 技术框架包含 ：**

- 参考车辆平台
- 参考硬件平台
- 开源软件平台
- 云服务平台
<a name="LPAp9"></a>
### 1. 参考车辆与硬件平台
[https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md)<br />
<br />_**[Apollo Hardware and System Installation Guide]**_ ─ Provides the instructions to install the hardware components and the system software for the vehicle:

- **Vehicle**:
   - Industrial PC (IPC)
   - Global Positioning System (GPS)：确定所处位置
   - Inertial Measurement Unit (IMU)： 惯性测量装置：测量车辆的运动和位置
   - Controller Area Network (CAN) card 控制区域网络：车辆的内部通信网络
      - 发送加速，制动，转向信号
   - GPS Antenna
   - GPS Receiver
   - Light Detection and Ranging System (LiDAR)：360度点云数据
   - Camera：捕获图像数据（计算机视觉处理）
   - Radar：分辨率低，但是经济实惠，适用于各种天气，照明条件，特别擅长测量其他车辆的速度
> ![](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Hardware_overview_3_5.png?raw=true#align=left&display=inline&height=642&margin=%5Bobject%20Object%5D&originHeight=642&originWidth=1676&status=done&style=none&width=1676)**Hardware/ Vehicle Overview**
> **![](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Hardware_connection_3_5_1.png?raw=true#align=left&display=inline&height=1142&margin=%5Bobject%20Object%5D&originHeight=1142&originWidth=2042&status=done&style=none&width=2042)
> **Hardware Connection Overview**
> 

> **
**<br />

- **Software**:
   - Ubuntu Linux
   - Apollo Linux Kernel
   - NVIDIA GPU Driver



> ![](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Apollo_3_5_software_architecture.png?raw=true#align=left&display=inline&height=882&margin=%5Bobject%20Object%5D&originHeight=882&originWidth=1732&status=done&style=none&width=1732)
> **Software Overview - Navigation Mode**

<a name="NJBna"></a>
### 2. 开源软件栈

1. **实时操作系统（ROTS）**
1. **运行时框架**
1. **应用程序模块层**

![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1599537233172-33398cc1-8444-43a1-ba73-dc74a624336b.png#align=left&display=inline&height=123&margin=%5Bobject%20Object%5D&name=image.png&originHeight=245&originWidth=1791&size=570648&status=done&style=none&width=895.5)

- **实时操作系统（ROTS）**
   - Ubuntu+Apollo
- **运行时框架**
   - 操作环境 ROS定制版（改进三块：**共享内存，去中心化 数据兼容性** ）
      1. 共享内存（一次写入，多次读取）
         - ![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1599538474868-f1ebc50f-da72-4448-bff4-e360130d81cb.png#align=left&display=inline&height=24&margin=%5Bobject%20Object%5D&name=image.png&originHeight=711&originWidth=1509&size=449111&status=done&style=none&width=51)
      2. 去中心化（ROS Master）（解决单点故障问题）
         - 将所有节点放在一个公共域中
      3. 数据兼容性 ROS Message
         - 使用protobuf的接口语言代替原生ROS消息
   - 在RTOS上运行的软件框架
- **应用程序模块层**
   - 各种模块：MAP引擎，定位，感知等等
   - ![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1599538837949-90cae8cd-f8d2-4503-9e92-e9e7f4d298c4.png#align=left&display=inline&height=311&margin=%5Bobject%20Object%5D&name=image.png&originHeight=622&originWidth=911&size=214132&status=done&style=none&width=455.5)



<a name="06BCm"></a>
### 3. 云服务

- 不仅仅提供数据，也提供了很多应用程序

**Cloud Services**

- HD Map
- Simulation（仿真环境平台）
- Data Platform：[http://apolloscape.auto/scene.html](http://apolloscape.auto/scene.html)
- Security
- OTA（over-the-air） update:无线更新
- DuerOS： 智能语音系统


<br />
<br />

<a name="6OBXL"></a>
### 4. Apollo Github库

<br />
<br />[Technical Tutorials](https://github.com/ApolloAuto/apollo/blob/master/docs/technical_tutorial/README.md)
