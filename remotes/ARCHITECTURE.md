# Remote 遥控器模块架构设计

## 1. 类层次结构

```
Remote (基类)                         ← dvc_remote.h / .cpp
├── public:
│   ├── remote_alive_status          存活状态
│   ├── Init()                       UART 初始化 + RTOS 任务创建
│   ├── AlivePeriodElapsedCallback()  掉线检测（50ms 轮询）
│   └── UartRxCpltCallback()         UART 接收回调 → DataProcess()
├── private:                          ← 内部实现，子类不可见
│   ├── flag_, pre_flag_             掉线检测滑动窗口
│   ├── last_raw_all_                按键边缘检测状态（每实例独立）
│   └── toggle_output_               Toggle 按键保持状态
├── protected:
│   ├── DataProcess()                [纯虚] 协议解析
│   ├── ClearData()                  [纯虚] 掉线数据清零
│   ├── Process_Keyboard_Toggle()    [虚] 按键 Toggle 模式处理
│   ├── Task()                       掉线检测任务循环
│   └── TaskEntry()                  FreeRTOS 静态入口
│
├── RemoteDjiVT03                    ← dvc_remote_vt03.h / .cpp
│   ├── output_ : RemoteVT03OutputData  公开
│   └── raw_data_ : RemoteVT03RawData*  私有
│
└── RemoteDjiDR16                    ← dvc_remote_dr16.h / .cpp
    ├── output_ : RemoteDR16OutputData  公开
    └── raw_data_ : RemoteDR16RawData   私有
```

### 设计模式：模板方法（Template Method）

基类 `Remote` 在 `UartRxCpltCallback()` 和 `AlivePeriodElapsedCallback()` 中定义了数据处理的骨架：
- 收到数据 → `flag_++` → 调用 `DataProcess(buffer)`（子类实现）
- 周期性检测 → 比较 `flag_` → 掉线则调用 `ClearData()`（子类实现）

子类只需实现纯虚方法，无需关心 UART、任务调度、掉线检测等基础设施。

## 2. 文件结构

```
Device/remotes/
├── dvc_remote.h          基类 Remote 声明（公共类型、枚举）
├── dvc_remote.cpp        基类 Remote 实现
├── dvc_remote_vt03.h     VT03 遥控器子类声明 + 数据结构
├── dvc_remote_vt03.cpp   VT03 协议解析实现
├── dvc_remote_dr16.h     DR16 遥控器子类声明 + 数据结构
├── dvc_remote_dr16.cpp   DR16 协议解析实现
└── ARCHITECTURE.md       本设计文档
```

### 包含依赖

```
dvc_remote.h
├── bsp_uart.h            UART 硬件抽象
├── FreeRTOS.h            RTOS
├── cmsis_os2.h           CMSIS-RTOS2 API
└── alg_math.h            数学工具

dvc_remote_vt03.cpp
├── dvc_remote_vt03.h
│   └── dvc_remote.h
└── app_gimbal.h          K_PITCH, C_PITCH 等俯仰常量

dvc_remote_dr16.cpp
├── dvc_remote_dr16.h
│   └── dvc_remote.h
├── app_gimbal.h          K_PITCH, C_PITCH 等俯仰常量
└── <algorithm>           std::clamp
```

**设计原则**：基类头文件不依赖应用层（`app_gimbal.h`），各子类 cpp 按需包含所需常量。

## 3. 数据结构

### 3.1 公共类型（定义于基类）

| 类型 | 说明 |
|------|------|
| `RemoteKeyStatus` | 按键状态：FREE / PRESS |
| `RemoteAliveStatus` | 存活状态：DISABLE / ENABLE |
| `RemoteKeyboard` | 16 位键盘位域（w/s/a/d/shift/ctrl/q/e/r/f/g/z/x/c/v/b） |
| `RemoteMouse` | 鼠标：x/y/z 位移 + 左右键 |

### 3.2 VT03 数据流

```
UART DMA → buffer
    ↓
RemoteDjiVT03::DataProcess()
    ├── 校验帧头 (0xA9 0x53)
    ├── 解包 11bit 通道 + 开关量
    ├── 俯仰通道 → pitch 弧度（K_PITCH, C_PITCH 标定）
    ├── 鼠标数据累积 + 限幅
    └── Process_Keyboard_Toggle() 键盘处理
    ↓
output_  ← 消费端（Robot::Task / UART 回调）
```

**`RemoteVT03OutputData`**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `remote.chassis_x/y` | uint16_t | 底盘摇杆（0-2047） |
| `remote.rotation` | uint16_t | 旋转通道 |
| `remote.thumbwheel` | uint16_t | 拨轮 |
| `remote.pitch` | float | 俯仰弧度 |
| `remote.all` | uint8_t | 开关量打包（pause/cns/fn1/fn2/trigger） |
| `mouse` | RemoteMouse | 鼠标数据 |
| `keyboard` | RemoteKeyboard | 键盘数据 |

### 3.3 DR16 数据流

```
UART DMA → buffer
    ↓
RemoteDjiDR16::DataProcess()
    ├── 位解包 11bit 通道
    ├── 俯仰通道 → pitch 弧度
    ├── 鼠标增量累积
    └── Process_Keyboard_Toggle() 键盘处理
    ↓
output_  ← 消费端（UART 回调 → CAN 转发至底盘 MCU）
```

**`RemoteDR16OutputData`**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `remote.switch_l/r` | uint8_t | 三段开关位置 |
| `remote.chassis_x/y` | float | 底盘归一化值 |
| `remote.rotation` | float | 旋转通道 |
| `remote.pitch` | float | 俯仰弧度 |
| `mouse` | RemoteMouse | 鼠标数据 |
| `keyboard` | RemoteKeyboard | 键盘数据 |

## 4. 关键设计决策

### 4.1 为何不引入统一输出结构体？

VT03 和 DR16 的物理层和数据格式差异较大：

| 维度 | VT03 | DR16 |
|------|------|------|
| 通道类型 | uint16_t（原始 11bit） | float（归一化） |
| 开关量 | fn1/fn2/pause/cns/trigger | switch_l/switch_r（三段开关） |
| 拨轮 | 有 | 无 |
| 鼠标 | 含鼠标中键 | 仅左右键 |

将所有字段强行塞入一个结构体会导致语义混乱（DR16 的 switch_l=1 不代表 fn1 被按下）。消费代码始终知晓具体遥控器类型，分开定义更清晰。

### 4.2 按键 Toggle 模式

操作模式由 `KEYBOARD_MODE`（`0xBFC0`）定义：bit 为 1 表示 Toggle 模式（按一次切换状态），bit 为 0 表示 Normal 模式（按下即触发，松手复位）。

```
按键位:  w  s  a  d  shift ctrl  q  e  r  f  g  z  x  c  v  b
模式:    T  T  T  T  N     N    T  N  N  N  N  N  N  N  N  N
```

Toggle 状态每个 `Remote` 实例独立维护（`last_raw_all_`, `toggle_output_` 为成员变量），避免多遥控器共存时互相干扰。

### 4.3 掉线检测机制

```
UartRxCpltCallback() 每收到一包数据 → flag_++
AlivePeriodElapsedCallback() 每 50ms：
    if (pre_flag_ == flag_) → 掉线 → ClearData() + remote_alive_status = DISABLE
    else → remote_alive_status = ENABLE
    pre_flag_ = flag_
```

掉线阈值约 50ms（一个任务周期）。掉线时各子类 `ClearData()` 将输出置为安全值。

## 5. 耦合分析与优化方向

### 5.1 耦合现状分析

当前继承关系存在多重耦合问题，基类 `Remote` 绑定了过多具体实现：

```
Remote 基类
├── 耦合点①: STM32 HAL UART     ← Init(UART_HandleTypeDef*, ...)
├── 耦合点②: FreeRTOS/CMSIS-RTOS2 ← Init() 内 osThreadNew()
├── 耦合点③: 固定掉线周期 (50ms)  ← Task() 内写死
├── 耦合点④: 固定的Toggle按键映射  ← KEYBOARD_MODE 宏
├── 耦合点⑤: 纯虚ClearData()      ← 子类必须实现
└── 耦合点⑥: 不感知输出数据        ← 子类 output_ 类型任意
```

#### 5.1.1 传输层耦合（耦合点①）

```cpp
// dvc_remote.h
void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

// dvc_remote.cpp
void Remote::Init(...) {
    uart_init(huart, callback_function, rx_buffer_length);  // 直接调用 BSP
    ...
}
```

**问题**：基类直接依赖于 STM32 HAL 的 `UART_HandleTypeDef` 和 BSP 层的 `uart_init()`。这导致：
- 整个继承树绑定到特定硬件，无法切换传输层（USB、CAN、SPI、网络）
- 无法在 PC/Linux 上做单元测试（无 STM32 HAL）
- 违反**依赖倒置原则（DIP）**：高层模块不应依赖低层模块，两者都应依赖抽象

#### 5.1.2 任务调度耦合（耦合点②）

```cpp
void Remote::Init(...) {
    ...
    osThreadNew(Remote::TaskEntry, this, &kRemoteTaskAttr);  // CMSIS-RTOS2 API
}
```

**问题**：基类直接调用 RTOS API 创建任务：
- 绑定到 CMSIS-RTOS2 / FreeRTOS，迁移到其他 RTOS 或裸机需改基类
- 任务栈大小（512）、优先级（osPriorityNormal）在基类硬编码，子类无法自定义
- `static TaskEntry` + `this` 模式是 C 风格包裹，类型安全性不足

#### 5.1.3 生命周期耦合（耦合点③⑤）

```cpp
void Remote::Task() {
    for(;;) {
        AlivePeriodElapsedCallback();  // 基类：检测flag变化
        osDelay(50);                   // 周期硬编码
    }
}

void Remote::AlivePeriodElapsedCallback() {
    if(pre_flag_ == flag_) {
        ClearData();                   // 纯虚：跳转子类清理输出
    }
}
```

**问题**：掉线检测逻辑在基类、数据清理在子类，两者被虚函数绑定：
- 掉线检测周期固定 50ms，子类和消费者无法调整
- `ClearData()` 是纯虚，所有子类必须实现（即使不需要特殊清理）
- 依赖方向不清晰：基类通过纯虚调用子类 → 基类依赖于子类的实现

#### 5.1.4 键盘处理耦合（耦合点④）

```cpp
// dvc_remote.cpp
#define KEYBOARD_MODE   0xBFC0

void Remote::Process_Keyboard_Toggle(...) {
    // 所有遥控器共享相同的 toggle 按键映射
    uint16_t toggle_mask = KEYBOARD_MODE;
    ...
}
```

**问题**：Toggle 按键的映射在所有 Remote 子类间共享：
- VT03 和 DR16 共享同一套按键 Toggle 配置
- 若某遥控器需要不同的 toggle 映射，必须重写 / 修改基类
- 但这种情况下共享是合理的（键盘硬件布局相同），属于可接受的偶合

#### 5.1.5 输出数据耦合（耦合点⑥）

```cpp
// 消费端代码直接依赖具体子类的 output_ 类型
robot_.remote_vt03_.output_.remote.fn2           // VT03 独有
robot_.remote_dr16_.output_.remote.switch_l       // DR16 独有
robot_.remote_vt03_.output_.remote.pitch           // 两者都有但类型不同
```

**问题**：没有统一的输出接口，消费端必须知道具体遥控器类型：
- VT03 的 `chassis_x` 是 `uint16_t`，DR16 是 `float`
- 无法编写对 `Remote` 基类编程的通用处理代码
- 新遥控器类型接入时，所有消费点都需要修改
- `output_` 直接暴露为 public 成员，破坏了封装

---

### 5.2 优化方向

针对上述耦合点，按解耦程度从低到高列出优化方案：

#### 5.2.1 方案一：抽象传输层（Strategy Pattern）

**目标**：解耦传输层（耦合点①），保留继承结构。

```
Remote                          Transport (抽象接口)
 ├── Init(Transport* trans) ──────▲─────────────
 ├── UartRxCpltCallback()         │             │
 └── ...                   UartTransport  TestTransport
```

```cpp
class Transport {
public:
    virtual void Init() = 0;
    virtual void Send(const uint8_t* data, uint16_t len) = 0;
    virtual void SetRxCallback(void (*cb)(uint8_t*, uint16_t)) = 0;
};

class Remote {
public:
    void Init(Transport* transport);  // 依赖注入
    // 不再出现 UART_HandleTypeDef
};

class UartTransport : public Transport {
    UART_HandleTypeDef* handle_;
    void Init() override { uart_init(handle_, ...); }
};
```

**收益**：
- Remote 不依赖 STM32 HAL，可切换传输层
- 可用 MockTransport 做 PC 端单元测试
- 新增传输方式（USB、CAN）只需新增 Transport 子类，Remote 不变

**代价**：
- 增加 Transport 虚函数表开销
- 需要修改 Init() 调用处的代码
- 仍保留继承结构，其他耦合点未解决

#### 5.2.2 方案二：策略化调度（分离 RTOS 依赖）

**目标**：解耦任务调度（耦合点②），允许不同调度策略。

```cpp
class TaskScheduler {
public:
    virtual void CreateTask(void (*entry)(void*), void* param,
                            const char* name, uint32_t stack_size) = 0;
    virtual void Delay(uint32_t ms) = 0;
};

class Remote {
public:
    void Init(Transport* transport, TaskScheduler* scheduler);
    // Init() 不再直接调用 osThreadNew
};

class FreeRtosScheduler : public TaskScheduler {
    void CreateTask(...) override { osThreadNew(entry, param, &attr); }
    void Delay(uint32_t ms) override { osDelay(pdMS_TO_TICKS(ms)); }
};
```

**收益**：
- 可配置任务栈大小、优先级
- 可在测试环境中替换为线程模拟或直接调用
- 与 CMSIS-RTOS2 解耦

**代价**：
- 增加虚函数调用层
- 对嵌入式项目来说可能过度设计（RTOS 很少切换）

#### 5.2.3 方案三：输出数据接口 + 统一访问

**目标**：解耦输出数据访问（耦合点⑥），为消费端提供统一入口。

```cpp
class RemoteOutput {         // 只读接口
public:
    virtual float   GetChassisX() const = 0;
    virtual float   GetChassisY() const = 0;
    virtual float   GetRotation() const = 0;
    virtual float   GetPitch() const = 0;
    virtual RemoteKeyboard GetKeyboard() const = 0;
    virtual RemoteMouse    GetMouse() const = 0;
};

class Remote {
public:
    virtual const RemoteOutput& GetOutput() const = 0;  // 纯虚
    // ...
};

class RemoteDjiVT03 : public Remote {
    RemoteVT03OutputData output_{};
    const RemoteOutput& GetOutput() const override { return output_adapter_; }
    // output_adapter_ 将 VT03 的 uint16_t 转为 float
};
```

**收益**：
- 消费端可对 `Remote` 基类编程，无需知道具体子类
- 新增遥控器类型不影响消费端代码
- 可编写跨遥控器的通用逻辑

**代价**：
- 虚函数访问开销（每次 GetChassisX() 都是间接调用）
- 原始值到归一化值的转换可能丢失精度
- 协议特有字段（如 VT03 的 fn1/fn2）仍需要向下转型访问

#### 5.2.4 方案四：桥接模式（Bridge Pattern）

**目标**：将协议解析与生命周期管理解耦，允许两者独立变化。

```
┌──────────────────────────────────────────────────┐
│                    Remote                          │
│  (传输 + 任务 + 掉线检测 + 键盘处理)                │
│         │                                           │
│         │ 协议委托                                   │
│         ▼                                           │
│  RemoteProtocol (抽象接口)                           │
│         ▲           ▲                                │
│         │           │                                │
│  VT03Protocol  DR16Protocol                          │
│  (帧校验、通道解包、  (位解包、标定、                 │
│   鼠标累积)         鼠标累积)                        │
└──────────────────────────────────────────────────────┘
```

```cpp
class RemoteProtocol {
public:
    virtual bool Decode(const uint8_t* buffer, uint16_t length) = 0;
    virtual void Clear() = 0;
    virtual ~RemoteProtocol() = default;
};

class Remote {
    RemoteProtocol* protocol_;
public:
    void Init(Transport* transport, RemoteProtocol* protocol, TaskScheduler* scheduler);
    void UartRxCpltCallback(uint8_t* buffer) {
        flag_++;
        protocol_->Decode(buffer, length);  // 委托给协议
    }
    void AlivePeriodElapsedCallback() {
        if(pre_flag_ == flag_) {
            protocol_->Clear();             // 委托给协议
        }
    }
};
```

**收益**：
- Remote 与具体协议完全解耦，可插入任意协议
- 协议可独立测试、独立复用
- 新增遥控器协议 = 新增 RemoteProtocol 子类，Remote 不变
- 遵循**开闭原则（OCP）**：对扩展开放、对修改封闭

**代价**：
- 增加一层间接调用
- 协议特有的输出数据访问路径变化
- 需要更复杂的初始化组装

#### 5.2.5 方案五：完整分层架构（推荐）

**目标**：综合以上方案，构建清晰的分层结构。

```
 ┌────────── 应用层 ──────────┐
 │   Robot / Init             │  ← 只关心遥控器输出，不关心协议
 │   (使用 RemoteOutput 编程)   │
 └──────────┬─────────────────┘
            │ 输出数据
 ┌──────────▼─────────────────┐
 │      Remote（核心层）        │  ← 管理生命周期、掉线检测、键盘处理
 │   Init(Transport, Protocol) │  ← 依赖注入，不直接创建
 │   GetOutput() → RemoteOutput│
 └──────┬──────────┬───────────┘
         │          │
   ┌─────▼──┐  ┌───▼──────────┐
   │Transport│  │RemoteProtocol│  ← 接口层，可独立替换
   │(UART/   │  │(VT03/DR16/   │
   │ USB/    │  │ CRSF/ SBUS)  │
   │ Test)   │  │              │
   └─────────┘  └──────────────┘
```

**各层职责**：

| 层 | 职责 | 变化原因 |
|------|------|---------|
| 应用层 | 消费遥控器数据、控制逻辑 | 机器人功能变化 |
| Remote 核心 | 数据接收、任务调度、掉线检测、键盘处理 | 调度策略、掉线策略 |
| Transport | 原始数据收发，硬件事务 | 硬件平台、通信方式 |
| Protocol | 协议解析、校验、状态计算 | 遥控器型号、协议版本 |

**组装方式**：

```cpp
// 在 Init() 或工厂函数中一次性组装
auto* transport = new UartTransport(&huart6, uart6_callback_function);
auto* protocol  = new VT03Protocol();
auto* scheduler = new FreeRtosScheduler("remote_task", 512);

Remote remote;
remote.Init(transport, protocol, scheduler);

// 消费端通过 RemoteOutput 接口访问
const auto& out = remote.GetOutput();
float pitch = out.GetPitch();
```

**完整收益评估**：

| 耦合点 | 当前 | 优化后 |
|--------|------|--------|
| ① STM32 HAL 依赖 | 基类直接调用 `uart_init()` | 注入 `Transport`，无关硬件 |
| ② RTOS API 依赖 | 基类直接调用 `osThreadNew()` | 注入 `TaskScheduler` |
| ③ 掉线周期固定 | 基类 `osDelay(50)` 硬编码 | 由 `TaskScheduler` 配置 |
| ④ Toggle 映射固定 | `KEYBOARD_MODE` 宏 | 可配置或由协议层提供 |
| ⑤ ClearData() 虚函数 | 子类必须实现 | 委托给协议层的 `Clear()` |
| ⑥ 输出数据类型不统一 | 消费端依赖具体子类类型 | 通过 `RemoteOutput` 接口访问 |

**适用性评估**：

| 维度 | 评价 |
|------|------|
| 可测试性 | Transport 和 Protocol 均可 Mock，支持 PC 端测试 |
| 可扩展性 | 新协议 = 新 Protocol 子类，新传输 = 新 Transport 子类 |
| 代码量 | 增加 3-5 个接口类，整体代码量增加约 30% |
| 运行时开销 | 多层虚函数调用，Cortex-M4 上约增加数十个时钟周期 |
| 学习成本 | 开发者需要理解策略模式、桥接模式、依赖注入 |
| 过度设计风险 | 对单一产品、不换 RTOS 的项目来说可能过度 |

#### 5.2.6 方案选择建议

| 场景 | 推荐方案 |
|------|---------|
| 仅需要 PC 测试 | 方案一（抽象 Transport） |
| 固定硬件、固定 RTOS | 维持现状 + 方案三（统一输出接口） |
| 需要接入多种遥控器协议 | 方案四（桥接模式） |
| 新产品平台，架构先行 | 方案五（完整分层） |
| 追求最小改动 | 方案一 + 方案三，保留继承 |

---

## 6. 重构记录

本次重构（2026-05-02）解决了以下问题：

| 问题 | 修改 |
|------|------|
| `Process_Keyboard_Toggle()` 使用 `static` 局部变量 | 改为成员变量，每实例独立 |
| `flag_`/`pre_flag_` 作用域过大 | `protected` → `private` |
| `RemoteDjiDR16::raw_data_` 可见性不一致 | `protected` → `private` |
| 头文件保护宏不规范 | 统一为 `DVC_REMOTE_H` 风格 |
| 文件头部注释错误（vt02/dji） | 修正与实际文件名一致 |
| 基类头文件依赖 `app_gimbal.h` | 下移至各子类 cpp |
| `RmoteVT03RawData` 拼写错误 | 修正为 `RemoteVT03RawData` |
