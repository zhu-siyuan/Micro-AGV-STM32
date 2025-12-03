# 🏎️ Micro-AGV-STM32 (工业级蓝牙小车)

![Status](https://img.shields.io/badge/Status-Developing-yellow) ![License](https://img.shields.io/badge/License-MIT-blue)

## 📖 项目简介 (Introduction)
本项目不仅仅是一个蓝牙小车，而是基于 **STM32F103** 主控，按照 **工业级 AGV (自动导引车)** 标准研发的桌面级移动机器人平台。

我们致力于实现：
- 🎯 **PID 闭环控制**：实现精准的直线行驶与原地转向。
- 📱 **蓝牙透传协议**：自定义数据包格式，具备失效保护机制。
- 🧱 **模块化架构**：硬件驱动层(HAL)与业务逻辑层解耦。

## 🛠️ 技术栈 (Tech Stack)
- **HW:** STM32F103C8T6, L298N/TB6612, 霍尔编码器电机
- **SW:** Keil MDK 6, STM32CubeMX, Visual Studio Code
- **Tools:** Altium Designer, SolidWorks

## 📅 开发进度 (Roadmap)
- [x] 项目立项与可行性分析
- [ ] 硬件选型与 PCB 绘制
- [ ] 3D 结构设计与打印
- [ ] 基础驱动开发 (Motor, UART)
- [ ] PID 算法调试
- [ ] 联调验收

## 👥 团队成员 (Team)
- **@zhu-siyuan** - Project Manager & Software
- **@min-huang-HW** - Hardware Engineer
- **@yuying-qin-MD** - Mechanical Engineer
