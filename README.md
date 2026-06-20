# Dust_Zephyr_Architecture_Modules

硬件设备模块封装层。在驱动层基础上组合算法和通讯能力，对外提供完整设备接口。

- **imu/** — 惯性测量单元（采样、加热控温、姿态解算）
- **motors/** — 电机驱动（DJI、DM）
- **powermeter/** — 功率计
- **remotes/** — 遥控器接收机（DR16、VT12、VT13）

模块可独立配备线程，也可由 thread 层组合使用。

详见 [ARCHITECTURE.md](ARCHITECTURE.md)。
