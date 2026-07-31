# modules/ - 设备模块层

`modules/` 位于底层驱动和项目线程之间。它把一个设备所需的驱动访问、
协议解析、状态管理和必要的算法组合成可以被项目直接使用的设备能力。

模块层解决的是“设备如何成为一个可用对象”，不解决“当前项目如何组织这些
对象”。项目线程负责选择模块、调用模块接口、建立线程间数据流，并决定具体
机器人业务行为。

## 目录结构

```text
modules/
├── imu/
│   ├── devices/       芯片级设备实现，例如 BMI088、ICM42688P
│   └── drivers/       IMU 管理、采样、加热控温、处理和校准
├── motors/
│   ├── dji/           DJI 电机及反馈协议
│   └── dm/            DM 电机及反馈协议
├── powermeter/        功率计模块
└── remotes/
    ├── dr16/          DJI DR16 协议
    ├── sbus/          SBUS 协议
    ├── vt12/          VT12 协议
    └── vt13/          VT13 协议
```

## 模块职责

| 模块 | 组合的底层能力 | 对外提供 |
| --- | --- | --- |
| `imu/` | SPI、PWM、定时器、姿态算法、稳定性识别 | 原始加速度和角速度、温度、姿态、校准状态 |
| `remotes/` | UART DMA、协议解析、串口探测 | 遥控器输入快照、连接状态、协议类型 |
| `motors/` | CAN、反馈帧解析、状态缓存 | 电机反馈快照和发送接口 |
| `powermeter/` | CAN、功率反馈帧解析 | 电压、电流、功率和在线状态 |

模块的公共头文件应表达设备能力和调用契约。芯片寄存器、协议字段和
具体外设句柄应尽量留在模块内部，避免项目线程直接依赖设备实现细节。

## 运行模型

模块不是单一的线程模型。当前代码中存在三种使用方式：

1. **模块自带运行线程**

   IMU 和遥控器需要持续采样、解析或状态切换，模块内部可以注册自己的
   线程。线程由 `REGISTER_THREAD()` 进入统一启动链，项目线程只消费模块
   提供的数据接口。

2. **中断回调驱动模块**

   电机和功率计主要在 CAN 接收回调中解析反馈帧，将最新状态写入缓存。
   这类模块不因为“属于设备”就额外创建线程，发送动作由上层线程调用。

3. **由项目线程组合的无状态或轻状态能力**

   一些处理、控制或转换只需要被项目线程周期性调用。模块提供接口和数据
   结构，不负责业务线程的周期、优先级和调度策略。

## 典型数据链

### IMU

```text
SPI / PWM / timer
        ↓
imu/devices + imu/drivers
        ↓
ImuManager
        ↓
project/thread/imu
        ↓
topic/imu_to 或其他业务消费者
```

IMU 的芯片实现负责寄存器和原始数据读取，驱动层负责采样组织、校准、
温度控制和姿态处理，项目线程负责把模块接入当前系统的数据契约。

### 遥控器

```text
UART DMA
    ↓
remotes protocol adapter
    ↓
Remote
    ↓
topic/remote_to
    ↓
project/thread/chassis + project/thread/gimbal
```

遥控器模块可以通过协议注册和探测机制支持多个接收机协议，但项目线程
不应直接解析 DR16、SBUS、VT12 或 VT13 的字节布局。

### 电机和功率计

```text
CAN RX callback
       ↓
motor / powermeter state cache
       ↓
project/thread/chassis 或 gimbal
       ↓
topic/to_can_tx
       ↓
project/thread/can
       ↓
CAN TX
```

接收回调只负责快速识别和更新状态；控制计算、功率限制和发送节奏属于
项目线程或算法层。

## Kconfig 与 CMake

模块是否存在由两层共同决定：

1. `modules/Kconfig` 暴露功能开关并声明依赖，例如 IMU 对姿态算法、PWM、
   PID、定时器和对应通信能力的选择关系。
2. `modules/CMakeLists.txt` 根据 `CONFIG_*` 把公共驱动、设备实现和协议
   实现加入编译。

因此，新增模块不能只添加 `.cpp` 文件。至少要检查：

```text
modules/Kconfig
modules/CMakeLists.txt
modules/<module>/include or public headers
modules/<module>/implementation
project/Kconfig
project/CMakeLists.txt
```

其中 `project/` 的配置决定当前产品是否使用模块，模块自身的 Kconfig 和
CMake 决定模块内部哪些实现参与构建。

## 与其他层的边界

| 层 | 负责内容 | 不应承担 |
| --- | --- | --- |
| `drivers/` | 单一外设或总线访问 | 设备协议组合和机器人业务 |
| `modules/` | 设备能力、协议和状态模型 | 当前项目线程编排 |
| `algorithm/` | 可复用计算和控制算法 | 直接管理具体外设生命周期 |
| `topic/` | 线程间消息和数据契约 | 解析设备原始协议 |
| `project/` | 板级选择、线程和业务组合 | 重复实现模块内部协议 |
| `init/` | 启动阶段、注册表遍历和公共分发 | 设备功能实现 |

## 新增模块的最小流程

1. 先确定模块的公共能力和调用者，不从项目线程的临时字段反推接口。
2. 把设备访问放入 `modules/<name>/devices` 或 `modules/<name>/drivers`，
   把协议适配器放入对应协议目录。
3. 在 `modules/Kconfig` 中声明开关、依赖和可选实现。
4. 在 `modules/CMakeLists.txt` 中加入公共实现和条件实现。
5. 如果模块需要持续运行，使用本地 `REGISTER_INIT()` / `REGISTER_THREAD()`
   注册，不修改顶层启动器。
6. 如果模块使用 CAN、远程协议或其他注册表，确认对应链接段由链接脚本
   保留，并检查回调上下文中的工作量。
7. 在 `project/` 中用线程或适配器接入模块，明确数据发布和消费方向。
8. 更新模块接口说明，并用配置搜索和静态路径检查确认新文件确实进入构建。

## 推荐阅读

- 模块边界、线程模型和注册机制：[ARCHITECTURE.md](ARCHITECTURE.md)
- 模块配置入口：[Kconfig](Kconfig)
- 模块编译入口：[CMakeLists.txt](CMakeLists.txt)
- 项目线程装配：[../project/README.md](../project/README.md)
- 系统启动层：[../init/README.md](../init/README.md)
