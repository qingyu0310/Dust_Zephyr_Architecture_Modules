# Remote — 遥控器接收机

## 概述

遥控器接收机模块负责从 UART（DMA 空闲中断）接收遥控器数据，自动探测当前连接的遥控器协议类型，锁定后持续解码并发布到 `zbus`。支持多种遥控器协议共存，无需手动配置。

## 架构分层

```
┌───────────────────────────────────────────┐
│                Remote 类                    │
│   状态机：Detecting → Locked → 循环解码     │
│   线程内运行，50ms 轮询 UART 信号量          │
└──────────┬────────────────────────────┬────┘
           │                            │
     HandleDetecting              HandleLocked
     ┌──────────────┐            ┌──────────────┐
     │ 驻留协议       │            │ while(有帧)   │
     │ hit 积累      │            │ Decode → pub  │
     │ need_hits 达标 │            │ fail_count++  │
     │ → Locked      │            │ UnlockLimit→   │
     │ retry 超标切   │            │ ResetDetect   │
     │ → 下一协议     │            └──────────────┘
     └──────────────┘
```

## 核心设计

### 1. 协议注册 — `REGISTER_REMOTE`

```cpp
#define REGISTER_REMOTE(RemoteType, frame_size_, priority_, need_hits_, name_) \
    static RemoteType kRemoteProtocol_##name_;                                   \
    static const remote::RemoteEntry kRemoteEntry_##name_                        \
    __attribute__((used, __section__(".remote"))) = {                            \
        #name_, frame_size_, &kRemoteProtocol_##name_, priority_, need_hits_     \
    }
```

用法（在协议 .cpp 末尾）：

```cpp
REGISTER_REMOTE(Dr16Protocol, kFrameSizeDR16, remote::Priority::Low, 3, dr16);
```

- 每个协议只需要一行注册，由链接器自动收集到 `.remote` 段
- `__remote_start` / `__remote_end` 提供遍历范围
- 增删协议不需要修改 Remote 核心代码

### 2. 协议基类 — `RemoteProtocol`

```cpp
class RemoteProtocol {
public:
    virtual bool Validate(const uint8_t *buffer, uint8_t len);
    virtual bool Decode(const uint8_t *buffer, uint8_t len,
                        topic::remote_to::Message &pub);
protected:
    uart_config line_cfg_ {};   // 各协议独立的 UART 配置
};
```

每个协议子类在构造函数中初始化 `line_cfg_`：
- 波特率（SBUS 100000、DR16 100000 等）
- 校验位（SBUS 偶校验、DR16 偶校验）
- 停止位（SBUS 2位、DR16 1 位）

切换协议时 `SwitchProto()` 自动应用对应配置。

### 3. 自动探测 — `HandleDetecting`

**流程：**

```
选择第一个协议 → SwitchProto（配 UART）
         │
    收到完整一帧
         │
    ┌────┴────┐
    │ Validate │
    └────┬────┘
   通过 ←┴→ 失败
    │         │
  hits++   retry++
    │         │
    ├ hits >= need_hits ──→ Locked（锁定）
    │
    ├ retry >= need_hits ──→ 切下一个协议
    │
    └ 不够 → 继续驻留
```

**关键规则：**

| 参数 | 含义 |
|------|------|
| `need_hits` | 锁定所需连续命中次数。值越大越难误锁 |
| `retry` | 当前协议连续失败次数。`>= need_hits` 时切换 |
| 单协议场景 | 直接探测即可，没有多协议竞争代价 |

**为什么 Validate 失败后丢整帧（Consume），不是丢 1 字节：**

空闲中断触发时 DMA 收到的是完整一帧。Validate 失败不是字节偏移问题，而是协议不匹配。丢掉整帧等下一帧更快。

### 4. 锁定解码 — `HandleLocked`

锁定后每帧调用一次 `Decode`，成功则解包 + 发布 `zbus`，失败则 `fail_count++`。连续失败达到 `kUnlockFailLimit`（5 次）回到探测。

**解码性能：** 一次 HandleLocked 约 1.6μs（480MHz 下 765 cycles）。

### 5. 断连处理

```
有数据 → 无数据：打一次 "lost XXX"，last_valid_ms 置 0
持续无数据：    沉默，不重复打 log
无数据 → 有数据：打一次 "reconnect XXX"，恢复解码
```

用 `last_valid_ms` 自身作为哨兵值（`== 0` 表示已打过 losing log），不需要额外的标志位。

### 6. 超时归零

失去信号时发布全零数据到 `zbus`，但不 `ResetDetect()`。遥控器关机后重新开机时，Locked 状态的 HandleLocked 自动恢复解码，不需要重新探测。

### 7. 架构对比

| 方案 | 优点 | 问题 |
|------|------|------|
| **函数指针表** | 简单直观，无 C++ 虚函数开销 | 每类协议要写 validate/decode 函数表，可扩展性差 |
| **虚基类（当前）** | 协议状态内聚（line_cfg + Validate + Decode），注册一行 | 虚函数间接调用 |
| **策略模式** | 运行时切换 | 过度设计，嵌入式不需要 |

选虚基类的理由：
- 每个协议的 UART 配置自然放在协议对象里
- 注册宏把实现和声明集中到协议 .cpp 末尾，不污染头文件
- 虚函数调用 < 10ns 级别，对比 SwitchProto 的 1ms busy_wait 可忽略

## 目录结构

```
remotes/
├── README.md             ← 本文件
├── remote.hpp            Remote 类声明、RemoteProtocol 基类、REGISTER_REMOTE
├── remote.cpp            状态机实现
├── protocol_base.hpp     公用类型（Mouse/Keyboard/normChannel/processChannel）
├── dr16/
│   └── dr16.cpp          DR16 协议：Validate + Decode + REGISTER_REMOTE
├── sbus/
│   └── sbus.cpp          SBUS 协议
├── vt12/
│   └── vt12.cpp          VT12 协议
└── vt13/
    └── vt13.cpp          VT13 协议
```

## 如何新增一个遥控器协议

1. 新建目录 `remotes/xxx/`，创建 `xxx.cpp`
2. 继承 `remote::RemoteProtocol`，实现 `Validate()` 和 `Decode()`
3. 构造函数初始化 `line_cfg_`（波特率、校验、停止位）
4. 文件末尾 `REGISTER_REMOTE(XxxProtocol, frame_size, priority, need_hits, xxx);`
5. 在 `project/thread/Kconfig` 的 `TRD_REMOTE` 下 `select MOD_DEV_REMOTE_XXX`
6. 在 `modules/CMakeLists.txt` 加上 `zephyr_library_sources` 编译

不需要改 `remote.hpp`、`remote.cpp`、协议遍历逻辑。

## 数据流

```
UART DMA 空闲中断
    │ 完整一帧
    ▼
k_sem_give → Task 循环
    │ uart_->Read()
    ▼
ProcessChunk → frame_buf_
    │ Dispatch()
    ▼
┌──────────┬──────────┐
│ Detecting │  Locked  │
│  Handle___│ Handle___│
│  Detecting│  Locked  │
└─────┬─────┴────┬────┘
      │           │ Validate / Decode
      ▼           ▼
    Consume → zbus_chan_pub
                 │
                 ▼
           topic::remote_to::Message
```

## 依赖

| 模块 | 用途 |
|------|------|
| `uart` | DMA 接收 + 空闲中断 + 线参数配置 |
| `thread` | 线程启动 |
| `topic/remote_to` | zbus 发布消息类型 |
