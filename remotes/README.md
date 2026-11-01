# remotes/ — 遥控器接收模块

协议解析、自动探测、锁定、失联处理，以及**用户区可覆盖的映射后处理**。

---

## 1. 数据流

```text
UART ──> Decode（协议解析）
            ↓  OutputData（协议原生结构）
               摇杆归一化 / 鼠标归一化 / 原始按键位 / 拨杆档位
       DefaultFillOutputData（框架默认映射）
               键盘长按-切换判定 + 灵敏度 + 键位映射
            ↓  Message 已填好
       FillOutputData（__weak 后处理，用户区按需覆盖）
            ↓
       ValidateOutput（框架兜底：钳位 + NaN 归零 + 枚举越界修正）
            ↓
       zbus 发布
```

**解析层只做归一化，不做灵敏度、不做按键语义。** 灵敏度系数、哪个键走长按哪个走切换、键位怎么映射，全在映射层。

**默认函数永远执行，弱函数只追加后处理。** 用户不写弱函数 = 空实现 = 行为与框架默认完全一致。

---

## 2. 协议注册

每个协议在自己的 `.cpp` 末尾一行：

```cpp
REGISTER_REMOTE(Vt13Protocol, kFrameSizeVT13, remote::Priority::High, 3, vt13);
```

宏生成静态协议实例 + `RemoteEntry`，塞进 `.remote` 链接段。`Remote` 遍历 `__remote_start..__remote_end` 自动探测。

**新增协议 = 写一个 `.cpp` + 一行 `REGISTER_REMOTE` + Kconfig/CMake 各一行。** 不需要改任何公共头文件。

| 协议 | 帧长 | 波特率 | 优先级 |
|---|---|---|---|
| dr16 | 18 | 100000 / Even | Low |
| sbus | 25 | 100000 / Even / 2 stop | High |
| vt12 | 16 | 115200 / None | Medium |
| vt13 | 21 | 921600 / None | High |

---

## 3. 用户区覆盖映射

### 3.1 最小覆盖：只改一个键

在自己的 `modules/remotes/` 里写同名同签名、**不带 `__weak`** 的定义：

```cpp
#include "vt13.hpp"

namespace vt13 {

/**
 * @brief VT13 后处理：自动瞄准改右键
 * @param in   协议输出
 * @param out  已被默认映射填好的业务消息
 */
void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    out.autoaim_ctrl = in.mouse.right ? topic::remote_to::StartMode::On : topic::remote_to::StartMode::Off;
}

} // namespace vt13
```

链接器遇到强弱两份同符号定义时选强的，框架那份被丢弃。

**不用调 `DefaultFillOutputData`** —— 它已经跑过了，`out` 里是完整的默认映射结果。

### 3.2 带状态：长按、单击切换

状态直接放弱函数里的 `static`：

```cpp
namespace vt13 {

void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    static bool aiming     = false;      // 瞄准开关
    static bool last_right = false;      // 右键边沿检测

    if (in.mouse.right && !last_right) {
        aiming = !aiming;
    }
    last_right = in.mouse.right;

    out.autoaim_ctrl = aiming ? topic::remote_to::StartMode::On : StartMode::Off;
}

} // namespace vt13
```

`REGISTER_REMOTE` 生成的是全局唯一实例，单实例架构下 `static` 就是正确的状态容器。

### 3.3 改灵敏度

`OutputData.mouse` 是**已归一化的 `[-1,1]`**，解析层不做灵敏度。映射层乘系数再钳位：

```cpp
namespace vt13 {

void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    out.yaw   = ClampAxis(in.mouse.x * 60.0f + in.ch.yaw);      // 水平灵敏度翻倍
    out.pitch = ClampAxis(in.mouse.y * 3.0f  + in.ch.pitch);    // 垂直灵敏度
}

} // namespace vt13
```

`ClampAxis` 定义在 `protocol_base.hpp`，`vt13.hpp` 已经 include 了它。

> 也可以不自己钳——`ValidateOutput` 会兜底。但 `ValidateOutput` 是给"写坏了"兜底的，正常路径自己钳更清楚。

### 3.4 装配

`modules/CMakeLists.txt`：

```cmake
if(CONFIG_DUST_MOD_DEV_REMOTE)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/remote_map_vt13.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes)
endif()
```

复用框架 `DUST_MOD_DEV_REMOTE` 门禁，**不新增 Kconfig 符号**。

---

## 4. 键盘键的两种模式：长按 / 切换

`KeyboardState::Process()` 用 `keyboard_mode` 位掩码区分每个键的行为：

| 模式 | 行为 | `keyboard_mode` 对应 bit |
|---|---|---|
| **长按** | 按住才是 1，松手变 0 | 0 |
| **切换** | 按一下翻转，松手保持 | 1 |

### 4.1 位定义

`keyboard_mode` 的每个 bit 对应 `Keyboard` union 里的一个键：

| bit | 键 | bit | 键 |
|---|---|---|---|
| 0 | `w` | 8 | `r` |
| 1 | `s` | 9 | `f` |
| 2 | `a` | 10 | `g` |
| 3 | `d` | 11 | `z` |
| 4 | `shift` | 12 | `x` |
| 5 | `ctrl` | 13 | `c` |
| 6 | `q` | 14 | `v` |
| 7 | `e` | 15 | `b` |

### 4.2 设定方式

**键盘处理在映射层，不在解析层。** `Decode` 只把原始按键位填进 `OutputData.keyboard`，长按/切换的判定由 `DefaultFillOutputData` 里的 `KeyboardState` 完成。

**框架默认（改 `kKeyboardMode` 常量）**：

```cpp
void Vt13Protocol::DefaultFillOutputData(const OutputData& od, Message& pub)
{
    constexpr uint16_t kKeyboardMode = 0;    // 全键长按
    // 让 f(bit9) 和 v(bit14) 走切换：
    // constexpr uint16_t kKeyboardMode = (1u << 9) | (1u << 14);

    static KeyboardState keyboard_state_ { kKeyboardMode };

    Keyboard kb {};

    keyboard_state_.Process(kb, od.keyboard);
    ...
```

三个协议各自独立配置：

| 文件 | 位置 |
|---|---|
| `dr16/dr16.cpp` | `Dr16Protocol::DefaultFillOutputData()` |
| `vt12/vt12.cpp` | `Vt12Protocol::DefaultFillOutputData()` |
| `vt13/vt13.cpp` | `Vt13Protocol::DefaultFillOutputData()` |

> `sbus` 没有键盘，不涉及。

**用户区覆盖（推荐）**：在自己的 `FillOutputData` 里自己做，不用框架那套：

```cpp
namespace vt13 {

void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    static KeyboardState kb_state { (1u << 14) };   // v 键切换

    Keyboard kb {};

    kb_state.Process(kb, in.keyboard);

    out.supercap_ctrl = kb.v ? topic::remote_to::StartMode::On : topic::remote_to::StartMode::Off;
}

} // namespace vt13
```

`in.keyboard` 是**原始按键位**，用户区想用什么规则都行——甚至完全不用 `KeyboardState`，自己写边沿检测。

### 4.3 机制

```cpp
uint16_t trigger     = current_raw.all & (~last_raw_all);   // 本帧新按下的键
uint16_t toggle_mask = keyboard_mode;                       // bit=1 的键走切换

toggle_output.all   ^= (trigger & toggle_mask);             // 切换键：按一次翻转
uint16_t normal_mask = ~toggle_mask;

current_output.all   = (toggle_output.all & toggle_mask)    // 切换键：用翻转后的值
                     | (current_raw.all & normal_mask);     // 其余键：用当前原始值
last_raw_all = current_raw.all;
```

### 4.4 注意

- **`keyboard_mode` 是位掩码，不是索引。** `keyboard_mode = 9` 表示 bit 0 和 bit 3（`w` 和 `d`）切换，不是"第 9 个键"。
- **`toggle_output` 是 `Process()` 内部维护的状态**，不要在外部改它。
- 切换模式下，第一次按下会翻转成 1，再按一次翻回 0 —— 状态由 `KeyboardState` 自己保存，跨帧有效。

---

## 5. 协议头文件暴露什么

以 `vt13.hpp` 为例：

| 内容 | 是否暴露 | 理由 |
|---|---|---|
| `SwitchCNS` 枚举 | ✅ | 输出侧取值语义，用户按 `cns` 分支需要可读常量 |
| `Switch` / `OutputData` | ✅ | `FillOutputData` 的入参类型，用户必须看到字段名 |
| `FillOutputData` 声明 | ✅ | 用户照着写同名同签名的覆盖 |
| `Mouse` / `Keyboard` / `KeyboardState` | ✅ 间接暴露（`protocol_base.hpp`） | `OutputData` 的字段类型，用户处理键盘需要 |
| `RawData`（packed 位域） | ❌ 留 `.cpp` 类内 private | 原始帧字节布局 = 协议解析知识 |
| `kSoF1` / `kMinChannel` 等常量 | ❌ 留 `.cpp` 类内 private | 同上 |
| `IsValidChannel()` 等校验 | ❌ 留 `.cpp` 类内 private | 协议合法性判断，不是业务映射 |
| `DefaultFillOutputData` | ❌ 留 `.cpp` 类内 private | 框架默认实现，用户不需要看 |
| `ValidateOutput` / `ClampAxis` | 在 `protocol_base.hpp`，但协议 `.hpp` **不 include 它** | 兜底校验，用户区不可见 |

---

## 6. 状态所有权

| 状态 | 归属 |
|---|---|
| `version` | **框架**，`HandleLocked` 每帧 `pub_.version + 1` |
| `last_valid_ms` / `lost` 标志 / 失联判定 / 失联清零 / 切协议复位 | **框架** |
| 本帧消息起点 | **框架**，局部 `Message frame_msg {}`，解码成功才赋给 `pub_` |
| 键盘 toggle（`keyboard_state_`） | **映射层**（`DefaultFillOutputData` 内 `static`；用户覆盖时归用户区） |
| 长按计时、瞄准开关、灵敏度、键位 | **映射层**（`DefaultFillOutputData` 内 `static`；用户覆盖时归用户区） |

**映射可以有状态，但不能拥有链路有效性。** 用户覆盖忘了写 `version++` 或忘了清上次的 `shoot_ctrl`，系统行为不该因此出错——所以 `version` 由框架自增，每帧从空消息 `frame_msg {}` 完整构造。

---

## 7. weak 覆盖生效的前提

### 6.1 必须满足

**框架源码直接进 app target**。`modules/CMakeLists.txt` 里全部是 `target_sources(app PRIVATE ...)`，不是 `zephyr_library()` 静态库。

若是归档库 `.a`，链接器可能因"无人引用"不抽取那个 `.o`，弱定义根本不参与链接，覆盖静默失效。

### 6.2 验证

```bash
# 最终符号选中了哪份：W=弱，T=强
nm build/zephyr/zephyr.elf | grep FillOutputData
```

看到 `T` = 用户区那份生效；看到 `W` = 只有框架默认。

### 6.3 关于 LTO

**当前工程未开 LTO**（`CONFIG_LTO` / `-flto` 均无），只需查 6.2 那条。

将来若开 LTO，追加查调用点是否被内联：

```bash
objdump -d build/zephyr/zephyr.elf | sed -n '/<.*Decode.*>:/,/^$/p' | grep -i filloutput
```

风险场景：默认定义与调用点在同一个 TU（`vt13.cpp` 里 `Decode` 调 `FillOutputData`），LTO 可能把弱版本内联进 `Decode`。此时强定义赢了符号解析，但调用点已被内联——覆盖静默不生效。

---

## 8. 输出校验

`ValidateOutput` 在 `FillOutputData` 之后调用，兜底修正用户区写坏的值：

| 情况 | 处理 |
|---|---|
| 连续量越界（如 `yaw = 5.0f`） | 钳到 `±1.0f` |
| `NaN` | 归零（`NaN` 无法钳位） |
| 枚举越界（如 `static_cast<StartMode>(7)`） | 归到安全默认值 |
| 只写坏一个字段 | 其余字段不受影响，**不丢弃整帧** |

**钳位不是清零**：用户区放大灵敏度后超出 `[-1,1]` 是正常用法，清零会让"打满方向"变成"不动"。

---

## 9. 已知限制

**只支持单实例。** `REGISTER_REMOTE` 生成的是 `static` 全局协议实例，`Remote` 也只有一个。`modules/Kconfig` 里的 `DUST_MOD_DEV_REMOTE_DUAL` 是没被引用的遗留符号。

---

## 10. 目录结构

```text
remotes/
├── remote.hpp / remote.cpp      探测、锁定、失联、发布
├── protocol_base.hpp            通用工具：归一化、钳位、输出校验
├── dr16/
│   ├── dr16.hpp                 OutputData + FillOutputData 声明
│   └── dr16.cpp                 协议实现 + DefaultFillOutputData + __weak FillOutputData
├── sbus/  （同构）
├── vt12/  （同构）
└── vt13/  （同构）
```
