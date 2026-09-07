# OpenFleX-Wheeled-Humanoid 快速导航
**全球首款 27 自由度开源轮式双臂机器人**

[English](./README.md) | 中文

---

[官方文档](http://docs.openarmx.com/) | [GitHub 组织](https://github.com/orgs/OpenFleX-Wheeled-Humanoid)

![OpenFlex 封面](./image/cover.png)

OpenFlex 是由成都长数机器人有限公司开发的开源全身人形机器人平台，基于 ROS 2 构建，集成双臂、移动底盘、升降台和头部系统，覆盖从机器人描述、底层驱动、VR遥操作到自主导航的完整技术栈。本页汇总了平台核心软件包的关键信息，帮助开发者快速定位所需模块。

---

## 平台功能展示

| 全身模型 | 底盘模型 | 建图定位 | 自主导航 |
|:---:|:---:|:---:|:---:|
| ![全身URDF](./image/openflex-urdf.gif) | ![底盘模型](./image/Chassis%20Model.gif) | ![建图](./image/Mapping.gif) | ![导航](./image/Navigation.gif) |
| **升降台URDF** | **升降台启动** | **升降台工具** | **头部URDF** |
| ![升降台URDF](./image/Lift-Slide-urdf.gif) | ![升降台启动](./image/Lift-Slide-bring.gif) | ![升降台工具](./image/Lift-Slide-tool.gif) | ![头部URDF](./image/hend_urdf.gif) |
| **头部启动** | **头部VR控制** | **头部视觉** | **外骨骼遥操作** |
| ![头部启动](./image/hend_bring.gif) | ![头部VR](./image/hend_vr.gif) | ![头部视觉](./image/Head-Vision.gif) | ![外骨骼](./image/Exoskeleton.gif) |
| **整机GUI** | **管理器GUI** | **整机启动** | **VLA模块** |
| ![整机GUI](./image/openlex-gui.png) | ![管理器](./image/openflex-manager-gui.gif) | ![整机启动](./image/openflex-bring.gif) | ![VLA模块](./image/openflex-vla.gif) |

---

## 场景应用

| 精确抓取 | 机器人厨师 | 超市拣选 | 工具分类 |
|:---:|:---:|:---:|:---:|
| ![精确抓取](./image/Precision_Grasping.gif) | ![机器人厨师](./image/robot_chef.gif) | ![超市拣选](./image/supermarket_picking.gif) | ![工具分类](./image/Tool_Sorting.gif) |
| **射箭** | **折叠衣物** | **低位物体稳定抓取** | **3D视觉手眼标定** |
| ![射箭](./image/archery.gif) | ![折叠衣物](./image/fold_clothes.gif) | ![低位物体稳定抓取](./image/Stable_Grasp_of_Low-ground_Objects.gif) | ![3D视觉手眼标定](./image/3D_Vision_Hand-Eye_Calibration.gif) |

---

## 包索引

```
openflex_all/
├── openflex_drivers/                    # 驱动包（.deb、.whl）
├── openflex_maps/                       # 地图文件
├── openflex_models/                     # 模型/权重文件
└── openflex_ws/                         # ROS 2 工作空间
    └── src/
        ├── OpenFlex/                    # 项目文档和安装脚本
        ├── openflex_integrated/   # 全身集成
        │   ├── openarmx_integrated_bringup/
        │   ├── openarmx_integrated_description/
        │   ├── openflex_gui/
        │   └── openflex_manager/
        ├── openflex_chassis/            # 底盘子系统
        │   ├── swerve_bringup/
        │   ├── swerve_hardware/
        │   ├── livox_ros_driver2/
        │   └── swerve_navigation/
        ├── openflex_lift_slide/         # 升降台子系统
        │   ├── lift_slide_hardware/
        │   ├── lift_slide_description/
        │   └── lift_slide_msgs/
        ├── openflex_armx/               # 双臂子系统
        │   ├── openarmx_bringup/
        │   ├── openarmx_hardware/
        │   ├── openarmx_description/
        │   └── openarmx_teleop_vr/
        ├── openflex_head/               # 头部子系统
        │   ├── openarmx_head_bringup/
        │   ├── openarmx_head_hardware/
        │   └── openarmx_head_description/
        ├── openflex_vr_bridge/          # VR姿态桥接
        ├── openflex_EXO/                # 外骨骼遥操作
        └── openflex_moveit_nav2/        # MoveIt2和Nav2
```

## 产品参数

### OpenFleX 轮臂机器人

| 参数 | 标准版 | Ultra 版 |
|:---:|:---:|:---:|
| **自由度** | 全身 27 关节<br/>头部 2 自由度<br/>左臂 8 自由度<br/>右臂 8 自由度<br/>升降 1 自由度<br/>底盘 8 自由度 | 全身 27 关节<br/>头部 2 自由度<br/>左臂 8 自由度<br/>右臂 8 自由度<br/>升降 1 自由度<br/>底盘 8 自由度 |
| **底盘类型** | 全向移动（四转四轮） | 全向移动（四转四轮） |
| **底盘负载** | ~120kg | ~120kg |
| **升降范围** | ~700mm | ~700mm |
| **单臂运动范围** | ~714mm | ~714mm |
| **双臂额定负载** ¹ | ~10.0kg | ~10.0kg |
| **双臂峰值负载** ² | ~24.0kg | ~24.0kg |
| **整机重量** | ~200.0kg | ~200.0kg |
| **通讯方式** | CAN 2.0 1Mbps | CAN 2.0 1Mbps |
| **电源组件** | 60Ah（30Ah × 2） | 60Ah（30Ah × 2） |
| **结构材料** | 铝合金 / 不锈钢 /3D 打印 | 铝合金 / 不锈钢 /3D 打印 |
| **域控制器** | X86 域控制器 | X86 域控制器 [配 RTX 3090 GPU] |
| **软件平台** | Ubuntu22.04, ROS2 Humble,<br/>robot_description, robot_hardware, etc | Ubuntu22.04, ROS2 Humble,<br/>robot_description,robot_hardware, etc |
| **感知系统** | 头部感知 双目 RGB 视觉 × 1 | 头部感知 RealSense D435i × 1<br/>腕部感知 RealSense D405 × 2<br/>底盘感知 Livox MID360S × 1<br/>+ RealSense D435i × 1 |
| **运动控制** | ROS2_control 实时关节控制<br/>MoveIt2 手臂规划控制 | ROS2_control 实时关节控制<br/>MoveIt2 手臂规划控制<br/>Nav2 底盘规划控制 |
| **具身智能** | **VLA**：Lerobot VLA 学习<br/>**RL**：Mujoco、NVIDIA Issac 强化学习<br/>**具身平台**：OpenClaw、DimOS | **VLA**：Lerobot VLA 学习<br/>**RL**：Mujoco、NVIDIA Issac 强化学习<br/>**具身平台**：OpenClaw、DimOS |
| **遥操作方式** | PICO4 ULTRA VR/ Meta Quest 全身遥操作 | PICO4 ULTRA VR/ Meta Quest 全身遥操作 |
| **选配组件** ³ | 灵巧手 /OEM/ODM | 灵巧手 /OEM/ODM |

**注释：**
1. 单臂额定负载是指机器人单臂在前平举状态下持续 1 分钟，J7 连杆能承受的最大重量。
2. 单臂峰值负载是指机器人单臂从垂直状态运动到前平举状态，保持 1 秒钟后返回，全过程持续约 3 秒钟，J7 连杆所能承受的最大重量。
3. 选配组件根据客户需求选择性配置，另提供具身智能机器人产品定制开发、OEM、ODM 等服务。

---

## 1. openflex_integrated

**概述**
OpenFlex 全身人形机器人集成系统，整合双臂、底盘、升降台和头部四大子系统，提供统一的启动、控制和管理接口。

**包含内容**
- `openarmx_integrated_bringup`：全身系统启动文件，支持真实硬件和仿真模式
- `openarmx_integrated_description`：完整机器人URDF/Xacro描述，包含所有子系统的运动学和动力学模型
- `openflex_gui`：基于Qt5的全身机器人图形化控制面板
- `openflex_manager`：系统管理器，提供各子系统的控制接口和状态监控
- 集成VR遥操作启动文件（`integrated_vr_teleop.launch.py`）
- 支持模块化启动：可独立启动各子系统或全身集成

**应用场景**
- 启动完整OpenFlex机器人进行全身控制
- VR沉浸式遥操作全身系统（双臂+底盘+头部）
- GUI图形化控制和系统状态监控
- 仿真环境下的全身运动规划和测试

**技术特性**
- ros2_control框架统一管理所有关节控制器
- 自动配置文件检测和创建（相机、雷达）
- 支持fake_hardware仿真模式
- 100Hz控制频率

---

## 2. openflex_chassis

**概述**
OpenFlex移动底盘与传感器子系统，采用四轮独立转向全向驱动，集成Livox MID-360激光雷达，支持FAST-LIO2 SLAM和Nav2自主导航。

**包含内容**
- `swerve_bringup`：底盘系统启动文件和控制器配置
- `swerve_hardware`：四轮Swerve Drive硬件接口（RS06转向电机+UM轮毂电机）
- `swerve_description`：底盘URDF模型
- `livox_ros_driver2`：Livox MID-360激光雷达驱动（已集成用户配置自动读取）
- `swerve_navigation`：SLAM建图和Nav2导航配置
  - FAST-LIO2实时建图（带PGO回环检测）
  - ICP定位
  - Nav2自主导航和避障
- CAN通信：can4（驱动电机）+ can5（转向电机）

**应用场景**
- 全向移动底盘的精确运动控制
- 室内环境SLAM建图和自主导航
- 激光雷达点云采集和处理
- 与双臂系统协同的移动操作

**技术特性**
- Swerve Drive算法实现全向移动
- 用户配置自动读取（`~/.openflex/lidar_config.yaml`）
- 雷达IP配置简化（只需输入最后2位数字）
- FAST-LIO2高精度实时建图
- PGO回环检测优化地图
- Nav2 DWB局部规划器

---

## 3. openflex_head

**概述**
OpenFlex 2自由度头部子系统，采用Robstride RS00电机驱动，支持VR头显跟踪和独立控制。

**包含内容**
- `openarmx_head_bringup`：头部系统启动文件（100Hz控制频率）
- `openarmx_head_hardware`：ros2_control硬件接口，支持MIT/CSP双控制模式
- `openarmx_head_description`：头部2-DOF URDF模型（yaw+pitch）
- `openarmx_head_teleop_vr_pico`：Pico VR头显跟踪控制
- `openarmx_head_visio_h264`：头部相机H.264视频流转发
- `openarmx_head_joint_slider_panel`：RViz2头部关节滑块控制面板
- CAN通信：can2（2个RS00电机）

**应用场景**
- VR头显方向跟踪，头部实时跟随
- 头部相机视频流实时转发到VR头显
- 独立的头部关节位置控制
- 与全身系统集成的协调运动

**技术特性**
- 关节限位：yaw ±90°，pitch ±90°
- 最大速度：33 rad/s
- 自动回零功能（启动时可选）
- 软限位平滑过渡
- 相对方向追踪模式

---

## 4. openflex_lift_slide

**概述**
OpenFlex单轴垂直升降台子系统，采用CANopen协议控制，提供腰部高度调节功能。

**包含内容**
- `lift_slide_hardware`：升降台CANopen硬件接口
- `lift_slide_description`：升降台URDF模型
- `lift_slide_msgs`：升降台消息定义
- `lift_slide_panel`：RViz2升降台控制面板
- CAN通信：can3（CANopen，Node ID 16）

**应用场景**
- 调节机器人腰部高度适应不同工作台面
- VR遥操作中的腰部升降控制
- 与底盘移动协同的全身运动

**技术特性**
- 行程范围：-0.650m ~ +0.300m
- 最大速度：0.10 m/s
- 位置精度：±0.5mm
- 限位开关保护
- Home开关自动回零

---

## 5. openflex_armx

**概述**
OpenFlex双7自由度机械臂子系统，基于openarmx-6.0_basic，采用Robstride电机驱动，支持多种遥操作方式。

**包含内容**
- 参考 [openarmx-6.0_basic](../openarmx-6.0_basic/README_CN.md) 完整文档
- 双臂CAN通信：can1（左臂）+ can2（右臂）
- 集成MoveIt2运动规划
- 支持VR、外骨骼、同构遥操作

**应用场景**
- 双臂协同操作和示教
- VR沉浸式双臂遥操作
- 轨迹录制和回放
- VLA数据采集

**技术特性**
- 7-DOF × 2臂配置
- Robstride电机CAN总线控制
- MIT/CSP双控制模式
- 重力补偿支持

---

## 6. openflex_vr_bridge

**概述**
Pico VR设备姿态桥接包，通过UDP接收VR手柄数据并发布为ROS 2话题，是VR遥操作的数据入口。

**包含内容**
- `pico_pose_bridge_node`（C++）：UDP端口5100监听，发布手柄位姿、按键、扳机等话题
- VR APK安装包（`../openflex_vr_apk/apk/pico/OpenFlex.apk` 或 `../openflex_vr_apk/apk/quest/openarmx-vr-quest.apk`）
- 支持双手柄6-DOF位姿跟踪
- 按键映射：A键（回零）、B键、扳机、握把

**应用场景**
- 作为所有VR遥操作的数据源
- 支持双臂VR IK遥操作
- 头部VR跟踪
- 底盘摇杆控制

**技术特性**
- UDP实时通信（~90Hz）
- 低延迟姿态数据传输
- 完整按键状态发布
- TF树可选发布

---

## 7. openflex_moveit_nav2

**概述**
OpenFlex的MoveIt2运动规划和Nav2导航集成包，提供全身协调的运动规划和自主导航能力。

**包含内容**
- 双臂MoveIt2配置
- Nav2导航参数配置
- 全身协调规划接口
- 移动操作集成

**应用场景**
- 双臂轨迹规划和避障
- 移动底盘自主导航
- 移动操作任务（导航+抓取）

**技术特性**
- MoveIt2 OMPL规划器
- Nav2 DWB控制器
- 全身碰撞检测
- 实时重规划

---

## 8. openflex_vla

**概述**
基于LeRobot框架的视觉-语言-动作（VLA）数据采集和模型训练包，支持多相机数据同步采集和ACT模型训练。

**包含内容**
- 数据采集脚本（`lerobot_record_openflex.py`）
- 4路RealSense相机同步
- VR遥操作数据记录
- ACT模型训练配置
- 推理部署脚本

**应用场景**
- VR遥操作示教数据采集
- 双臂操作数据集构建
- ACT模仿学习模型训练
- 策略模型在线推理

**技术特性**
- 多相机时间同步
- HDF5数据格式
- 支持LeRobot生态
- GPU训练加速

---

## 🚀 快速上手

### 1.1 编译构建

若你使用官方预配置主机，可跳过本节，直接阅读: [openflex-说明文档(基础)-CN.md](openflex-说明文档(基础)-CN.md)

---

## 🎮 VR遥操作功能

### A键回零机制
- **功能**：按下右手柄A键，双臂14个关节同时回到0°位置
- **特性**：
  - 平滑运动，每周期限制8°步长
  - 完成条件：所有关节误差<0.01rad
  - 可中途取消（再次按A键）
  - 夹爪状态保持

### 其他控制
- **扳机键**：双臂IK控制
- **摇杆**：底盘移动（启用时）
- **头部跟踪**：VR头显方向跟随

---

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
