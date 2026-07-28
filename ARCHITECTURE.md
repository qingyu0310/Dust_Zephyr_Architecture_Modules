# modules/ 架构说明

`modules/` 是 tflm 的设备能力封装层。

它位于 `drivers/` 和 `project/` 之间：

```text
project/thread/
    组合具体项目业务
        ↓
modules/
    提供可复用设备能力
        ↓
drivers/
    提供 UART、SPI、CAN、PWM 等硬件访问
```

算法通常从 `algorithm/` 提供：

```text
modules/
    负责设备状态、协议、采样和设备生命周期

algorithm/
    负责 PID、滤波、姿态、辨识和纯计算
```

模块层的目标不是把所有代码都变成“有线程的类”，而是把一个设备或设备能力封装成一个可被项目组合的对象。

有些模块需要持续运行，有些模块只需要响应中断，有些模块只需要被上层控制循环周期性调用。线程是否存在，应该由设备的运行模型决定，而不是由“每个模块必须有 Start()”这种形式规则决定。

---

## 一句话理解

`modules/` 负责回答：

> 一个具体的硬件设备，如何被封装成可以初始化、读取、控制、校验、发布或组合的能力对象？

当前模块层的核心边界可以概括为：

```text
驱动层提供硬件通道
    -> 设备模块封装协议和状态
        -> 项目线程决定多个模块如何协同
            -> topic 传递跨线程业务数据
```

模块可以拥有自己的线程，但线程必须属于模块自身的运行语义，而不是为了凑目录层次强行添加。

---

## 当前目录

```text
modules/
├── ARCHITECTURE.md
├── README.md
├── CMakeLists.txt
├── Kconfig
├── imu/
│   ├── drivers/
│   │   ├── imu.hpp / imu.cpp
│   │   ├── heater.hpp / heater.cpp
│   │   └── processor.hpp / processor.cpp
│   └── devices/
│       ├── imu_device_layer.hpp
│       ├── bmi088/
│       └── icm42688p/
├── remotes/
│   ├── remote.hpp / remote.cpp
│   ├── protocol_base.hpp
│   ├── dr16/
│   ├── sbus/
│   ├── vt12/
│   └── vt13/
├── motors/
│   ├── dji/
│   └── dm/
└── powermeter/
    ├── powermeter.hpp
    └── powermeter.cpp
```

当前模块类别：

| 模块 | 主要能力 | 运行方式 |
| --- | --- | --- |
| `imu/` | 原始采样、校准、加热、姿态解算 | 模块内部线程 |
| `remotes/` | UART 接收、协议探测、解码、zbus 发布 | 模块内部线程 |
| `motors/` | CAN 反馈解析、状态快照、控制帧打包 | CAN 回调 + 上层控制线程 |
| `powermeter/` | CAN 功率数据解析、状态快照 | CAN 回调 + 上层控制线程 |

---

## modules/ 的职责

### 它负责什么

模块层负责：

- 封装具体设备协议；
- 保存设备运行状态；
- 把原始硬件数据转换为工程量；
- 提供统一的设备接口；
- 管理设备初始化、复位、校验和错误状态；
- 组合底层 `drivers/`；
- 组合必要的 `algorithm/`；
- 根据设备运行模型决定是否拥有线程；
- 提供可被项目线程调用的同步接口；
- 通过 Kconfig 表达设备能力和底层依赖。

例如：

```text
Icm42688p
    -> SPI 寄存器访问
        -> 原始 accel/gyro/temp
            -> CalibSource 公共校准
                -> ImuManager 采样
                    -> Processor 姿态解算
                        -> topic::imu_to
```

再例如：

```text
DjiC620
    -> CanCpltRxCallback()
        -> 更新内部状态
            -> ReadAll()
                -> project/thread/chassis
                    -> PID / 功率控制
                        -> CAN TX topic
```

### 它不负责什么

模块层不应该负责：

- 某一台机器人完整业务策略；
- 底盘、云台、发射机构之间的协调；
- 项目级任务编排；
- 具体板卡的业务拓扑；
- 把所有设备强行放进一个总控类；
- 为所有设备创建统一格式的线程；
- 直接把原始寄存器访问暴露给项目层；
- 通过全局变量替代明确的设备接口。

模块可以拥有设备内部状态，但不应该知道“这个设备在整台机器人里要怎么决策”。

---

## 模块边界

| 管 | 不管 |
| --- | --- |
| 具体设备协议和状态管理 | 整机业务策略 |
| 设备初始化、复位、错误检查 | 项目级任务编排 |
| 原始数据到工程量的转换 | 多模块之间的控制决策 |
| 设备自身需要的持续运行逻辑 | 机器人模式状态机 |
| 设备专用线程或事件循环 | 所有模块统一强行线程化 |
| 与 `drivers/`、`algorithm/` 的组合 | 直接修改业务 topic 语义 |
| Kconfig 依赖和 CMake 裁剪 | 依赖某个具体项目目录 |

模块的边界可以用一句话表达：

> 模块负责把一个设备变得可用，项目线程负责决定多个可用设备如何一起工作。

---

## 模块的三种运行模型

当前模块层并不是“有线程”和“没线程”二选一，而是至少存在三种运行模型。

### 1. 主动数据源模块

主动数据源会持续从硬件获取数据，并且数据本身有稳定的时间节奏。

典型模块：

- IMU；
- Remote。

它们通常需要：

- 持续等待输入；
- 管理缓冲区；
- 维护协议或采样状态机；
- 在数据到达后完成一系列连续处理；
- 对超时、断连、重新连接作出处理；
- 向 topic 发布最新数据。

这类模块允许拥有自己的线程。

### 2. 被动设备对象

被动设备本身不需要主动运行循环。

典型模块：

- DJI 电机；
- DM 电机；
- PowerMeter。

它们通常由两个入口组成：

```text
硬件事件
    -> CanCpltRxCallback()
        -> 更新状态

上层控制线程
    -> ReadAll()
    -> PackCtrlFrame()
    -> 使用快照完成控制
```

这类对象不应该为了“模块完整”而创建线程。

### 3. 项目组合模块

有些逻辑不是单个设备的内部逻辑，而是多个设备共同参与的控制闭环。

典型场景：

- 底盘；
- 云台；
- 功率分配；
- 多电机同步；
- Remote 输入到多个执行机构的映射。

这类线程应该位于 `project/thread/`：

```text
project/thread/chassis/
    读取 Remote topic
    读取多个 Motor 快照
    执行运动学与 PID
    执行功率分配
    发布 CAN TX 数据
```

它不能被塞回单个电机模块，因为电机模块不应该知道其他电机、底盘坐标系和功率预算。

---

## 为什么 IMU 的 SPI 不放在基类

这是当前 IMU 架构中最重要的边界之一。

当前 IMU 继承关系大致是：

```text
Source
    ↓
CalibSource
    ↓
Icm42688p / Bmi088
```

职责分别是：

```text
Source
    统一设备生命周期和采样接口

CalibSource
    复用原始样本、单位换算、校准和 Flash 保存流程

Icm42688p / Bmi088
    负责具体芯片的寄存器协议和硬件访问
```

公共基类只要求子类提供：

```cpp
virtual bool ReadRaw(ImuRawSample& raw) = 0;
virtual float ConvertAccel(int16_t raw) const = 0;
virtual float ConvertGyro(int16_t raw) const = 0;
virtual float ConvertTemperature(int16_t raw) const = 0;
```

基类并不知道“原始样本是通过什么总线得到的”。

### 基类真正关心的是样本，不是总线

`CalibSource` 的公共流程是：

```text
ReadRaw()
    -> ConvertAccel / ConvertGyro / ConvertTemperature
        -> offset 校准
            -> scale 校准
                -> Sample
```

它需要的是一个 `ImuRawSample`，而不是一个 `Spi` 对象。

如果把 `Spi` 放进基类，基类就会变成：

```text
所有 IMU 都必须有一个 SPI
所有 IMU 都必须接受同一种 SPI 初始化方式
所有 IMU 都必须共享同一种寄存器访问假设
```

这会把公共数据处理层和具体硬件总线绑定起来。

现在的设计则是：

```text
基类只定义数据源接口
子类自己决定如何获得原始样本
```

这是更稳定的抽象边界。

### ICM42688P 和 BMI088 的总线拓扑并不相同

`Icm42688p` 当前包含：

```cpp
Spi spi_;
```

它通过一个 SPI 设备访问 ICM42688P，并且自己处理：

- `SEG_SEL` 寄存器段选择；
- SPI 模式；
- 连续寄存器读取；
- 读写缓冲区；
- WHO_AM_I 校验；
- 设备复位；
- 寄存器配置。

而 `Bmi088` 当前包含：

```cpp
Spi accel_;
Spi gyro_;
```

BMI088 的加速度计和陀螺仪是两个独立的 SPI 设备。

它需要自己管理：

- 加速度计 SPI 配置；
- 陀螺仪 SPI 配置；
- 两个从设备的初始化顺序；
- 两套寄存器地址；
- 两套芯片 ID；
- 两套复位和配置流程；
- 读取 accel、gyro 和 temperature 时如何组合。

如果把 SPI 资源放到 `CalibSource`：

```text
一个基类 SPI
    无法自然表达 BMI088 的两个 SPI 设备
```

为了适配 BMI088，就必须把基类设计成：

```cpp
Spi spi_a_;
Spi spi_b_;
```

这会把一个具体芯片的特殊拓扑污染到所有 IMU。

子类私有成员则自然表达了设备事实：

```text
ICM42688P
    一个设备
        一个 SPI 对象

BMI088
    两个子器件
        两个 SPI 对象
```

### 子类拥有 SPI，可以保持寄存器协议内聚

IMU 的 SPI 不只是“发送几个字节”。

每种芯片都可能不同：

- 读写位定义不同；
- dummy byte 数量不同；
- 大小端不同；
- 寄存器分页方式不同；
- 复位等待时间不同；
- 连续读取地址规则不同；
- 需要不同的片选或设备树配置；
- 初始化校验顺序不同。

`Icm42688p` 需要：

```text
SelectSegment()
ReadRegs()
WriteChecked()
SoftReset()
InitRegisters()
```

`Bmi088` 需要：

```text
ReadAccel()
ReadGyro()
InitAccel()
InitGyro()
WriteCheckedAccel()
WriteCheckedGyro()
```

这些函数都属于具体芯片协议。

如果 SPI 也放到基类，基类会逐渐出现各种“为了兼容不同芯片”的虚函数：

```text
SelectPage()
ReadRegister()
WriteRegister()
ReadAccel()
ReadGyro()
```

这样公共基类会越来越像一套芯片协议，而不是数据源抽象。

当前设计把这些细节限制在子类内：

```text
具体寄存器协议
    -> 具体设备子类私有

公共样本处理
    -> CalibSource 复用
```

### 子类私有 SPI 也保护了访问权限

SPI 成员放在子类 `private` 区域还有一个作用：防止上层越过设备接口直接访问寄存器。

上层只应该调用：

```cpp
source->Init();
source->Read(sample);
source->Calibrate();
```

而不应该调用：

```cpp
icm.spi_.Send(...);
```

这样可以保证：

- 寄存器协议不会泄漏到项目线程；
- 设备初始化状态不会被外部破坏；
- SPI 访问顺序由设备自身控制；
- 以后更换 ICM42688P 时，上层接口不变。

### 子类私有 SPI 有利于测试和替换

基类可以使用一个没有真实 SPI 的测试子类：

```cpp
class FakeSource final : public CalibSource
{
protected:
    bool ReadRaw(ImuRawSample& raw) override;
    float ConvertAccel(int16_t raw) const override;
    float ConvertGyro(int16_t raw) const override;
    float ConvertTemperature(int16_t raw) const override;
};
```

测试 `CalibSource` 时，不需要创建 SPI 设备。

这说明公共校准逻辑并不依赖真实硬件，而具体设备子类才负责把硬件接进来。

### 这不是“SPI 不重要”

SPI 仍然是重要的底层能力，只是它应该位于正确的位置：

```text
drivers/communication/stream/spi/
    提供通用 SPI 访问能力

IMU 设备子类
    持有一个或多个 Spi 对象
    组织具体寄存器事务

CalibSource
    不感知 SPI
```

SPI 放在子类私有，不是降低 SPI 的地位，而是避免把总线细节扩散到公共抽象。

---

## IMU 的模块分层

当前 IMU 不是一个单独类，而是由多个职责层共同组成。

```text
project/thread/imu/trd_imu.cpp
    项目启动适配
        ↓
imu::ImuManager
    数据源选择、加热、姿态、采样线程
        ↓
CalibSource
    原始样本公共处理、校准、工程量转换
        ↓
Icm42688p / Bmi088
    具体芯片 SPI 寄存器访问
```

旁路组件：

```text
heater::Heater
    PWM + PID + 稳定判据

attitude::Processor
    EKF 姿态更新

topic::imu_to
    跨线程姿态消息
```

### Source

`Source` 定义所有 IMU 数据源必须提供的能力：

```cpp
virtual bool Init() = 0;
virtual bool LateInit() = 0;
virtual bool Read(Sample& sample) = 0;
virtual bool Calibrate();
```

它不关心：

- 芯片型号；
- SPI 或 I2C；
- 寄存器分页；
- 设备树 alias；
- 加速度计和陀螺仪是否分成两个芯片。

### CalibSource

`CalibSource` 把多个 IMU 共用的流程集中起来：

- `ReadRaw()`；
- 原始数据转换；
- gyro offset；
- accel offset；
- scale；
- 温度转换；
- Flash 校准参数读取；
- Flash 校准参数保存；
- 静态校准采样。

子类只提供原始样本和转换规则。

### ImuManager

`ImuManager` 负责把一个具体数据源变成可运行的 IMU 系统：

- 遍历 `.imu` 注册段；
- 确认只注册一个 IMU；
- 初始化数据源；
- 初始化加热器；
- 选择正常、自动校准或自动辨识模式；
- 调用 `LateInit()`；
- 初始化姿态处理器；
- 启动采样线程；
- 更新加热器；
- 更新姿态；
- 准备 `topic::imu_to::Message`。

这个类属于设备能力管理，不属于某个具体机器人项目。

---

## 为什么 IMU 有线程

IMU 是一个主动数据源。

它不是“被调用一次才读取一次”的普通对象，而是一个持续产生时间序列数据的系统。

当前 `ImuManager::Task()` 每个周期执行：

```text
读取原始样本
    -> 计算真实 dt
        -> 更新加热器
            -> 更新姿态 EKF
                -> 准备发布消息
```

### IMU 有固定采样节奏

姿态解算依赖连续时间序列。

当前任务中会：

```cpp
source_->Read(sample_);
sample_.dt = kMeasureDT();
heater_.Update(sample_.temp);
attitude_.Process(sample_, pub_);
k_msleep(1);
```

这不是一个只在上层控制线程需要时才读取的设备。

如果把 IMU 读取放到 `project/thread/chassis` 或 `project/thread/gimbal`：

- 采样频率会被具体业务线程决定；
- 不同业务线程可能重复读取；
- 姿态解算会和底盘、云台调度耦合；
- 更换使用 IMU 的项目时，需要重写采样循环；
- 加热器更新和姿态更新时间可能失去统一节奏。

因此，IMU 自己维护采样、温控和姿态处理线程更自然。

### IMU 的线程包含一个完整内部闭环

IMU 的运行不是单一 `Read()`：

```text
传感器读取
    -> 温度采样
        -> 加热 PWM 更新
            -> 校准与单位换算
                -> 姿态 EKF
```

这些步骤之间有明确的同一帧关系。

例如：

- `sample.temp` 是当前采样周期的温度；
- heater 使用当前温度更新 PWM；
- attitude 使用当前样本和真实 dt；
- 后续 topic 应该对应同一轮采样结果。

把这些步骤拆到多个外部线程，会引入不必要的同步和时序问题。

### IMU 线程不等于项目线程

当前 `project/thread/imu/trd_imu.cpp` 仍然存在，但它是启动适配器：

```cpp
static ::imu::ImuManager imu_ {};

bool thread_init()
{
    return imu_.Init(::imu::ImuStartMode::AutoCalib);
}

bool thread_start()
{
    return imu_.Start(ThreadPrio::High);
}
```

这两个层次的职责不同：

```text
project/thread/imu
    决定当前项目使用什么启动模式、什么优先级

imu::ImuManager
    决定 IMU 自己如何采样、控温、解算和运行
```

所以项目层仍然保留项目装配权，模块层保留设备运行权。

---

## 为什么 Remote 有线程

Remote 也是主动数据源，但它和 IMU 的主动方式不同。

IMU 是周期采样，Remote 是连续 UART 字节流。

当前 Remote 需要处理：

- UART DMA 数据到达；
- 缓冲区拼接；
- 不同长度的帧；
- 多协议探测；
- 协议锁定；
- 解锁和重新探测；
- UART 参数切换；
- 输入超时；
- 断连时发布归零数据；
- zbus 发布。

这些逻辑天然适合集中在 Remote 自己的线程中。

### UART DMA 回调不应该承担完整协议解析

底层 UART DMA 负责接收数据并通过信号量通知：

```text
UART DMA / 空闲事件
    -> BipBuffer
        -> k_sem_give
            -> Remote::Task()
                -> uart_->Read()
                    -> ProcessChunk()
```

ISR 或底层回调只负责：

- 搬运数据；
- 更新接收缓冲；
- 唤醒线程。

Remote 线程负责：

- 按帧长度处理数据；
- 调用 `Validate()`；
- 调用 `Decode()`；
- 维护协议探测状态；
- 发布 topic；
- 处理超时和重连。

这样可以避免在中断上下文里做：

- 字节搬移；
- 字符串或协议解析；
- 虚函数调用；
- zbus 发布；
- 多协议状态机切换。

### Remote 内部是一个持续运行的状态机

Remote 的核心状态是：

```text
Detecting
    -> 逐个尝试注册协议
        -> Validate 命中
            -> hits 达到阈值
                -> Locked
```

锁定后：

```text
Locked
    -> Decode
        -> 发布有效遥控数据
```

连续失败或超时后：

```text
Locked
    -> fail_count 达到阈值
        -> ResetDetect
            -> Detecting
```

这个状态机需要长期保存：

- 当前锁定协议；
- 当前探测协议；
- 命中次数；
- 重试次数；
- 连续解码失败次数；
- 上次有效时间；
- 当前帧缓冲；
- 当前 UART 配置。

这些状态属于 `Remote` 对象自身，不应该分散到项目线程和中断回调里。

### Remote 的线程由模块持有，启动由项目装配

`Remote` 类内部拥有：

```cpp
Thread<1024 * 5> thread_;
```

并提供：

```cpp
bool Init(UartDma& uart);
bool Start(ThreadPrio prio);
```

项目线程只负责：

```text
选择 device tree 中的 remote_uart
    -> 配置 UART
        -> remote_.Init(rx)
            -> remote_.Start(...)
```

这是一种“模块拥有运行逻辑，项目拥有装配参数”的方式。

### Remote 不应该被拆成每个协议一个线程

协议文件只负责：

```cpp
Validate()
Decode()
GetLineCfg()
```

协议注册到 `.remote`：

```cpp
REGISTER_REMOTE(Dr16Protocol, ...);
REGISTER_REMOTE(SbusProtocol, ...);
REGISTER_REMOTE(Vt12Protocol, ...);
REGISTER_REMOTE(Vt13Protocol, ...);
```

真正的接收线程只有一个。

如果每个协议各自维护线程：

- 所有线程会竞争同一个 UART；
- 每个线程都会读取同一段数据；
- 探测和锁定难以集中管理；
- 切换波特率时会出现多个所有者；
- 协议增删会影响线程装配。

当前设计把复杂性收敛在 `Remote` 内部：

```text
协议是可插拔对象
线程是统一接收状态机
UART 是统一资源
```

这也是 Remote 允许独立线程的根本原因。

---

## 为什么 Motor 没有线程

电机模块的本质是：

```text
CAN 反馈协议对象
```

而不是一个自主运行的控制器。

当前 DJI 和 DM 电机对象提供：

- `Init()`；
- `CanCpltRxCallback()`；
- `ReadAll()`；
- 状态读取接口；
- 目标值设置接口；
- 控制帧打包接口；
- 掉线检查。

它们没有：

- `Thread<>`；
- `Start()`；
- 内部周期循环；
- 独立的 CAN 发送线程。

这是有意为之。

### 电机反馈是事件驱动的

电机反馈由 CAN 帧触发。

典型运行关系是：

```text
CAN RX
    -> DjiC620::CanCpltRxCallback()
        -> 更新角度、速度、电流、转矩
            -> seqlock 保护状态

project/thread/chassis
    -> ReadAll()
        -> 使用一致快照计算 PID
```

电机并不需要自己周期性地“检查有没有新数据”。

数据什么时候到达由 CAN 总线决定，回调就是最直接的更新入口。

如果再给每个电机创建线程，线程实际上只会做：

```text
周期唤醒
    -> 看看状态有没有更新
        -> 把状态交给别人
```

这会增加调度开销，却没有增加设备能力。

### 电机控制周期属于上层控制闭环

一个电机在不同项目里的控制方式可能不同：

```text
底盘电机
    速度环 / 力矩环 / 功率分配

云台电机
    角度环 / 速度环 / 力矩控制

测试电机
    开环转矩 / 参数辨识
```

如果电机自己拥有线程，它就必须知道：

- 控制周期是多少；
- 目标值从哪里来；
- 使用哪组 PID；
- 是否参与功率分配；
- 发送到哪条 CAN 总线；
- 与其他电机如何同步。

这些都不是电机协议本身的职责。

当前设计把电机控制放在项目线程：

```text
project/thread/chassis
    -> 同时读取多个电机
    -> 按底盘坐标系计算目标
    -> 统一执行 PID
    -> 统一进行功率分配
    -> 统一组帧发送
```

这样多个电机可以在同一个控制周期内被一致处理。

### 多电机同步要求统一的上层线程

底盘控制通常要求：

```text
同一周期读取所有电机状态
    -> 计算统一运动学目标
        -> 统一控制
            -> 统一发送
```

如果每个电机各自拥有线程：

- 不同电机的读取时刻不一致；
- PID 周期难以统一；
- 多电机功率分配需要跨线程同步；
- 底盘控制逻辑会反向依赖多个电机线程。

因此电机模块只提供一致快照：

```cpp
auto snap = motor.ReadAll();
```

由上层决定如何使用这个快照。

### seqlock 让电机可以无独立线程工作

电机状态可能在 CAN 回调中更新，而控制线程同时读取。

当前电机对象使用 `atomic_t seq_` 做 seqlock：

```text
写入开始
    -> seq 变为奇数
        -> 更新多个状态字段
            -> seq 变为偶数

读取开始
    -> 读取 seq
        -> 奇数则重试
            -> 复制完整快照
                -> 再检查 seq 是否变化
```

这样控制线程得到的是同一帧状态快照，而不是角度来自上一帧、速度来自下一帧的混合数据。

这正是“中断更新 + 上层读取”模型的配套同步机制。

### Motor 模块不持有 CAN 线程

CAN 总线通常会被多个设备共享：

- 多个 DJI 电机；
- 多个 DM 电机；
- PowerMeter；
- 其他 CAN 设备。

如果每个设备都创建自己的 CAN 线程：

- 每个线程都要知道总线；
- 每个线程都要处理接收队列；
- 总线分发逻辑会被拆散；
- 多从设备共享难以统一。

更合理的结构是：

```text
CAN 驱动 / 项目总线入口
    -> 按 CAN ID 分发
        -> 对应模块的 CanCpltRxCallback()
```

模块只负责解析自己收到的帧，不负责拥有整条总线。

当前 `project/apps/Irq_handlers` 提供了 CAN 接收分发基础设施，模块提供 `CanCpltRxCallback()` 作为设备协议入口。具体设备实例、CAN ID 和项目总线归属仍属于项目装配边界。

---

## 为什么 PowerMeter 没有线程

PowerMeter 和 Motor 的运行模型非常接近。

它是一个 CAN 状态接收对象：

```text
CAN 帧
    -> CanCpltRxCallback()
        -> 更新电压、电流、功率
            -> ReadAll()
                -> 底盘功率控制
```

它不需要主动采样。

### 功率计没有独立的周期动作

当前 PowerMeter 的工作只有：

- 接收 CAN 帧；
- 解析原始数值；
- 换算为工程量；
- 保存最新快照。

它没有：

- 自主控制输出；
- 周期性设备配置；
- 协议探测；
- 超时后必须独立发布的消息；
- 必须独立运行的滤波状态机。

因此没有必要维护一个 PowerMeter 线程。

### 功率数据的消费时机由底盘决定

底盘线程当前会在自己的控制周期中读取：

```cpp
SteerPwrMeter.GetPower();
DrivePwrMeter.GetPower();
```

然后交给功率控制器：

```text
电机状态
    -> 预测功率

PowerMeter 实测功率
    -> 修正功率模型

功率控制器
    -> 分配电流预算
```

功率计数据的意义取决于当前底盘控制周期。

因此让底盘线程消费功率计，而不是让功率计自己创建线程，更符合数据的使用关系。

### PowerMeter 线程会制造无意义的中间层

如果 PowerMeter 自带线程，可能会变成：

```text
CAN RX
    -> PowerMeter 线程
        -> PowerMeter 缓存
            -> Chassis 线程
                -> PowerCtrl
```

而当前设计是：

```text
CAN RX
    -> PowerMeter 快照
        -> Chassis 线程
            -> PowerCtrl
```

少一个线程，就少一层调度、消息传递和生命周期管理。

### 未来需要超时或滤波时怎么办

没有线程不代表 PowerMeter 永远不能增加时间逻辑。

如果以后需要：

- 掉线检测；
- 滑动滤波；
- 数据有效期判断；
- 多帧统计；

可以有三种选择：

1. 在 CAN 回调里只更新时间戳和原始数据；
2. 在使用它的底盘线程里完成判断和滤波；
3. 如果确实形成独立运行模型，再把维护逻辑提升为明确的系统任务。

关键不是“永远不加线程”，而是先判断这段逻辑属于谁。

当前 PowerMeter 的职责仍然只是：

```text
接收、转换、保存、提供快照
```

所以没有线程是正确的。

---

## 四类设备的线程判断表

| 设备 | 数据来源 | 是否主动运行 | 当前线程策略 | 原因 |
| --- | --- | --- | --- | --- |
| IMU | SPI 周期采样 | 是 | 模块内部线程 | 采样、温控、姿态解算有统一节奏 |
| Remote | UART 连续字节流 | 是 | 模块内部线程 | 需要缓冲、协议状态机、超时和发布 |
| Motor | CAN 反馈帧 | 否 | 无模块线程 | 回调更新，项目控制线程统一消费 |
| PowerMeter | CAN 测量帧 | 否 | 无模块线程 | 回调更新，底盘线程决定消费时机 |
| Chassis | Remote + 多电机 + 功率计 | 是 | `project/thread` | 多模块协同属于项目业务 |
| Gimbal | Remote + DM 电机 | 是 | `project/thread` | 控制闭环和设备组合属于项目业务 |

可以进一步压缩成：

```text
主动产生数据
    -> 模块可以拥有线程

被动响应硬件事件
    -> 回调更新 + 快照读取

多个设备共同决策
    -> project/thread 拥有控制线程
```

---

## 模块内部线程和 project/thread 的关系

当前工程采用的是“模块拥有运行逻辑，项目拥有启动装配”的混合方式。

### 模块内部负责

模块内部负责：

- 线程循环；
- 线程私有缓冲；
- 设备状态机；
- 设备内部的周期关系；
- 设备自身的超时和恢复；
- 设备内部的数据处理。

例如：

```cpp
class Remote final
{
public:
    bool Init(UartDma& uart);
    bool Start(ThreadPrio prio);

private:
    Thread<1024 * 5> thread_;
    void Task();
};
```

### project/thread 负责

项目线程适配器负责：

- 从设备树选择硬件；
- 创建或绑定驱动对象；
- 设置当前项目的配置；
- 选择启动模式；
- 选择线程优先级；
- 把模块接入当前项目的启动阶段。

例如：

```cpp
bool thread_init()
{
    static UartDma rx {};
    rx.Init(DEVICE_DT_GET(DT_ALIAS(remote_uart)), cfg);
    remote_.Init(rx);
    return true;
}

bool thread_start()
{
    return remote_.Start(ThreadPrio::High);
}
```

这比把设备树、项目参数、线程逻辑全部塞进模块更灵活。

---

## 模块注册与编译裁剪

### Kconfig

`modules/Kconfig` 负责表达模块能力和底层依赖：

```text
MOD_DEV_IMU
    -> FLT_QUATERNION
    -> TPC_IMU_TO
    -> DEV_PWM
    -> CTL_PID
    -> CTL_TIMER

MOD_DEV_IMU_BMI088
    -> COM_SPI

MOD_DEV_IMU_ICM42688P
    -> COM_SPI

MOD_DEV_REMOTE
    -> COM_UART_DMA
    -> TPC_REMOTE_TO

MOD_DEV_MOTOR_DJI
    -> COM_CAN

MOD_DEV_MOTOR_DM
    -> COM_CAN

MOD_DEV_POWERMETER
    -> COM_CAN
```

模块只声明自己需要什么。

项目层通过 `select` 选择模块：

```text
TRD_IMU
    -> MOD_DEV_IMU

TRD_REMOTE
    -> MOD_DEV_REMOTE

TRD_CHASSIS
    -> MOD_DEV_MOTOR_DJI
    -> MOD_CTL_POWER
    -> TPC_REMOTE_TO
```

这样设备模块不需要知道自己最终被哪台机器人使用。

### CMake

`modules/CMakeLists.txt` 根据 Kconfig 决定源文件是否进入编译：

```text
MOD_DEV_IMU
    -> 编译公共 IMU 管理、加热、姿态处理

MOD_DEV_IMU_BMI088
    -> 编译 BMI088 设备实现

MOD_DEV_IMU_ICM42688P
    -> 编译 ICM42688P 设备实现

MOD_DEV_REMOTE
    -> 编译 Remote 核心

协议开关
    -> 编译对应 DR16 / SBUS / VT12 / VT13
```

这让同一个模块层可以按项目裁剪：

```text
只用 ICM42688P
    -> 不编译 BMI088

只用 DR16
    -> 不编译其他协议

不用 PowerMeter
    -> 不编译功率计模块
```

---

## IMU 设备注册

IMU 使用 `.imu` 链接段注册具体设备：

```cpp
REGISTER_IMU(Icm42688p, icm42688p);
```

宏会生成：

```text
静态 Icm42688p 实例
    -> ImuEntry
        -> .imu
```

`ImuManager::InitSource()` 遍历：

```cpp
extern const imu::ImuEntry __imu_start[];
extern const imu::ImuEntry __imu_end[];
```

并要求：

```text
0 个 IMU
    -> 初始化失败

1 个 IMU
    -> 选中并初始化

多个 IMU
    -> 初始化失败
```

当前设计不是让项目线程手动写：

```cpp
Icm42688p imu;
imu.Init();
```

而是：

```text
设备实现本地注册
    -> 模块管理器统一发现
        -> 项目线程只决定启动模式和优先级
```

这使增加新 IMU 时不需要修改 `ImuManager` 的选择逻辑。

---

## Remote 协议注册

Remote 使用 `.remote` 链接段注册协议：

```cpp
REGISTER_REMOTE(Dr16Protocol, kFrameSizeDR16,
                remote::Priority::Low, 3, dr16);
```

协议子类负责：

- UART 线参数；
- 帧长度；
- `Validate()`；
- `Decode()`；
- 协议自身的数据转换。

Remote 核心负责：

- 遍历协议表；
- 自动探测；
- 协议锁定；
- 协议切换；
- 连续失败解锁；
- 超时归零；
- zbus 发布。

这体现了两种不同复杂度的分离：

```text
协议细节
    -> 一个协议一个类

接收状态机
    -> 一个 Remote 统一管理
```

增加协议时，不需要修改 Remote 状态机。

---

## Motor 的设备接口

Motor 模块的公共接口围绕“反馈快照”和“控制帧”设计。

### 反馈入口

```cpp
void CanCpltRxCallback(uint8_t* buffer);
```

它负责：

- 解析编码器；
- 计算多圈角度；
- 计算速度；
- 计算电流；
- 计算转矩；
- 解析温度或错误状态；
- 更新 seqlock。

### 读取入口

```cpp
Snapshot ReadAll() const;
```

控制线程一次读取完整快照。

### 控制出口

DJI 电机通常由上层准备发送数据槽。

DM 电机提供：

```cpp
PackCtrlFrame()
PackCmdFrame()
PackSetCtrlMode()
```

这些接口只负责把目标值转成协议帧，不负责决定何时发送。

发送时机由项目线程和 CAN TX 线程决定。

这保持了三个职责边界：

```text
Motor
    负责协议和状态

project/thread
    负责控制算法和发送策略

drivers/CAN
    负责总线访问
```

---

## PowerMeter 的设备接口

PowerMeter 的接口更简单：

```cpp
void CanCpltRxCallback(uint8_t* buffer);
Snapshot ReadAll() const;
```

它不拥有：

- CAN 总线；
- CAN 接收线程；
- 功率控制策略；
- 功率预算；
- 电机分组。

它只提供：

```text
最新分流电压
最新母线电压
最新电流
最新功率
```

底盘功率控制器再决定如何使用这些数据。

这让 PowerMeter 可以被不同项目复用：

```text
底盘功率分配
    -> 读取 PowerMeter

调试记录
    -> 读取 PowerMeter

功率辨识
    -> 读取 PowerMeter
```

模块不需要知道具体使用者。

---

## 数据一致性

Motor 和 PowerMeter 的状态可能在 CAN 回调中更新，同时被项目线程读取。

因此它们采用 seqlock 保护快照。

### 写入侧

```text
seq++
    -> 写入多个字段
        -> seq++
```

### 读取侧

```text
读取 seq
    -> 如果为奇数，重试
        -> 复制字段
            -> 再读 seq
                -> 变化则重试
```

这比每个字段单独读取更可靠。

例如 PowerMeter 的四个量：

```text
shunt_volt
bus_volt
current
power
```

应该来自同一帧，而不是跨越两个 CAN 回调。

模块层提供快照一致性，项目层负责判断：

- 数据是否超时；
- 数据是否合理；
- 数据是否参与控制；
- 数据是否需要降级。

---

## 线程归属的判断规则

新增模块时，可以按下面的问题判断是否需要线程。

### 问题一：设备是否主动产生连续数据

如果答案是“是”，考虑模块线程：

- IMU 周期采样；
- UART 连续接收；
- 协议状态机；
- 周期性发布。

### 问题二：硬件事件是否已经提供了天然入口

如果 CAN 或 GPIO 回调已经能直接更新状态，而且设备没有额外周期动作，不需要线程：

- 电机反馈；
- PowerMeter；
- 简单传感器状态。

### 问题三：控制逻辑是否需要多个设备同时参与

如果答案是“是”，线程通常放在 `project/thread/`：

- 底盘；
- 云台；
- 多电机同步；
- 功率分配；
- Remote 到执行机构的映射。

### 问题四：线程中的逻辑是否属于设备自身

如果线程只处理设备内部状态机，可以放在模块：

```text
Remote 协议探测
IMU 采样和姿态
```

如果线程需要知道多个设备和项目业务，应放在项目层：

```text
底盘运动学
云台控制
功率预算
```

---

## 常见错误设计

### 错误一：所有模块都必须有线程

这会导致：

- 电机每个实例一个线程；
- PowerMeter 一个线程；
- 简单设备也周期唤醒；
- 多设备控制被拆散；
- 线程数量和优先级难以管理。

正确做法是先判断设备运行模型。

### 错误二：把 SPI 放进所有 IMU 的公共基类

这会导致：

- 基类绑定 SPI；
- BMI088 双 SPI 拓扑难以表达；
- I2C 或其他总线难以扩展；
- 芯片寄存器协议向基类泄漏；
- 测试基类时必须构造硬件对象。

正确做法是：

```text
公共基类抽象数据源
具体子类持有具体总线
```

### 错误三：让 Motor 自己决定控制周期

电机不知道：

- 自己属于底盘还是云台；
- 当前使用位置、速度还是力矩控制；
- 其他电机的状态；
- 当前功率预算；
- 当前项目是否需要发送。

这些属于项目控制线程。

### 错误四：让 PowerMeter 自己计算功率分配

PowerMeter 只知道测量值，不知道：

- 哪组电机优先；
- 当前底盘总预算；
- 当前转向和行进的控制目标；
- 低功率时如何降级。

测量和策略必须分开。

### 错误五：让 Remote 协议类自己创建线程

协议类只应该解析协议。

如果 DR16、SBUS、VT12、VT13 各自拥有线程，就会破坏统一 UART 所有权和探测状态机。

正确做法是：

```text
协议类无线程
Remote 核心拥有一个线程
```

---

## 新增 IMU 的建议流程

新增一个 IMU 设备时，建议按下面的顺序。

### 1. 继承 `CalibSource`

```cpp
class NewImu final : public CalibSource
{
public:
    bool Init() override;
    bool LateInit() override;

protected:
    bool ReadRaw(ImuRawSample& raw) override;
    float ConvertAccel(int16_t raw) const override;
    float ConvertGyro(int16_t raw) const override;
    float ConvertTemperature(int16_t raw) const override;
};
```

### 2. 在子类中持有设备专用总线对象

```cpp
private:
    Spi spi_ {};
```

如果设备有多个从器件，就按真实拓扑持有多个对象：

```cpp
private:
    Spi accel_ {};
    Spi gyro_ {};
```

### 3. 把寄存器协议留在子类

包括：

- 设备复位；
- 芯片 ID；
- 寄存器配置；
- 数据读取；
- 读写校验；
- 地址和字节序。

### 4. 注册到 `.imu`

```cpp
REGISTER_IMU(NewImu, new_imu);
```

### 5. 在 Kconfig/CMake 中裁剪

新增设备开关，并只在启用时编译对应 `.cpp`。

### 6. 保持 `ImuManager` 不变

如果新增设备必须修改 `ImuManager`，通常说明设备差异没有被正确封装在 `Source` 或子类里。

---

## 新增 Remote 协议的建议流程

新增协议时：

1. 新建 `modules/remotes/xxx/xxx.cpp`；
2. 继承 `remote::RemoteProtocol`；
3. 设置 `line_cfg_`；
4. 实现 `Validate()`；
5. 实现 `Decode()`；
6. 使用 `REGISTER_REMOTE()`；
7. 在 Kconfig/CMake 中加入协议开关；
8. 不修改 Remote 核心线程和状态机。

协议实现不应该：

- 自己创建 UART；
- 自己读取 DMA；
- 自己发布多个 topic；
- 自己创建线程；
- 自己修改其他协议状态。

---

## 新增 Motor 或 PowerMeter 的建议流程

新增 CAN 设备模块时：

1. 定义设备配置；
2. 定义反馈 `Snapshot`；
3. 实现 `CanCpltRxCallback()`；
4. 使用 seqlock 保护多字段状态；
5. 提供 `ReadAll()`；
6. 在需要发送时提供帧打包接口；
7. 把 CAN ID、实例和总线归属留在 `project/`；
8. 不创建独立设备线程；
9. 让项目线程决定消费和控制周期。

如果设备确实需要周期性动作，例如：

- 周期性查询寄存器；
- 周期性发送心跳；
- 独立超时恢复；
- 设备自身状态机必须持续运行；

再重新评估是否需要线程。

---

## 最终判断

当前 `modules/` 的核心原则是：

```text
设备差异留在设备子类
公共数据流程放在公共基类
主动数据源可以拥有线程
被动 CAN 设备只提供回调和快照
多设备协同由 project/thread 负责
```

因此：

### IMU 的 SPI 放在子类私有

因为 SPI 设备数量、总线配置、寄存器协议和访问顺序都是具体芯片的事实。

公共基类应该只关心：

```text
原始样本
单位转换
校准
统一读取接口
```

### IMU 和 Remote 有线程

因为它们都是主动数据源：

```text
IMU
    周期采样 + 温控 + 姿态处理

Remote
    UART 流接收 + 协议状态机 + 超时发布
```

它们内部拥有连续运行逻辑，线程属于模块自身。

### Motor 和 PowerMeter 没有线程

因为它们是被动 CAN 设备：

```text
CAN 回调更新状态
    -> 上层控制线程读取快照
```

它们不应该知道项目控制周期，也不应该为每个设备重复创建线程。

这套设计把三种变化隔离开：

```text
换 IMU
    -> 改具体设备子类

换 Remote 协议
    -> 增加协议实现和注册项

换底盘或云台控制策略
    -> 改 project/thread
```

这正是 `modules/` 在 tflm 中的价值：

> 把设备变成可替换、可复用、可组合的能力对象，同时不替项目层决定设备必须如何运行。
