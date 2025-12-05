# 🏎️ Micro-AGV-STM32 (工业级桌面移动机器人)

![Status](https://img.shields.io/badge/Status-Developing-yellow) ![Hardware](https://img.shields.io/badge/Hardware-STM32F103-blue) ![License](https://img.shields.io/badge/License-MIT-green) ![Build](https://img.shields.io/badge/Build-Keil%20%7C%20VSCode-orange)

> **"不仅仅是玩具，而是遵循工业设计标准开发的 Micro-AGV 移动平台。"**

## 📖 项目简介 (Introduction)

本项目旨在研发一款**具备工业级研发标准的桌面级三轮全向移动小车 (Micro-AGV)**。
区别于市面上常见的“飞线+胶带”式玩具车，本项目严格遵循 **SOP (标准作业程序)**，采用 **V模型开发流程**，旨在验证从需求分析、GitHub 版本管理、PCB 集成设计到结构堆叠的全套正规军研发流程。

### 核心特性 (Key Features)
*   **🤖 闭环控制 (PID)**：摒弃开环控制，引入霍尔编码器反馈，实现速度与航向的精准修正。
*   **📡 工业级通信协议**：自定义定长数据帧（帧头+校验+帧尾），具备通信容错与失效保护机制。
*   **🏗️ 模块化软件架构**：基于 HAL 库，严格分层 (Drivers / Middlewares / Application)，解耦硬件细节。
*   **🛡️ 全包裹工业设计**：3D 打印外壳，隐藏式走线与电池仓设计，提升整机一体性。

---

## 🛠️ 系统架构 (System Architecture)

### 1. 硬件选型 (Hardware)
| 模块 | 型号/规格 | 选型理由 |
| :--- | :--- | :--- |
| **主控 MCU** | STM32F103C8T6 | 生态成熟，IO 资源满足 PWM/UART/编码器需求 |
| **电机驱动** | TB6612FNG | 双路 H 桥，高效率，低发热，体积小 |
| **动力系统** | JGA25-370 (或 N20) | **带霍尔编码器**，支持 AB 相测速 (PID 基础) |
| **电源系统** | 2×18650 (7.4V) | 配合 LM2596 稳压模块，抗干扰能力强 |
| **通信模块** | HC-05 蓝牙 | 串口透传，兼容性好 |

### 2. 软件架构 (Software)
本项目采用分层架构设计：
- **📂 Application (业务层)**: 状态机管理 (待机/遥控/急停)、指令解析。
- **📂 Middlewares (功能层)**: PID 算法库、运动学解算 (Kinematics)、电机控制逻辑。
- **📂 Drivers (驱动层)**: 基于 STM32 HAL 库封装 GPIO, PWM, UART, EXTI。

---

## 📡 通信协议 (Communication Protocol)

为保证通信可靠性，拒绝单字符控制。采用 **定长数据帧** 格式：
**波特率：** 115200

| 字节索引 | 定义 | 值/说明 |
| :--- | :--- | :--- |
| **0** | **帧头 (Header)** | `0xA5` (固定，用于数据对齐) |
| **1** | **功能字 (Func)** | `0x01`: 遥控模式 <br> `0x02`: PID参数整定 |
| **2** | **数据 X** | X轴速度 (int8, -127~127) |
| **3** | **数据 Y** | Y轴速度 (int8, 预留给全向轮) |
| **4** | **数据 Omega** | 转向速度 (int8, -127~127) |
| **5** | **校验位 (Check)** | Sum Check (前5字节之和的低8位) |
| **6** | **帧尾 (Tail)** | `0x5A` (固定) |

> **失效保护 (Fail-Safe)**: 若连续 500ms 未收到合法数据帧，小车将自动急停。

---

## 📂 目录结构 (Directory Structure)
```text
Micro-AGV-STM32/
├── Docs/               # 项目文档 (可行性报告, Datasheets)
├── Hardware/           # 硬件工程 (Altium Designer/嘉立创 EDA)
│   ├── Schematic/      # 原理图
│   └── PCB/            # PCB源文件 & Gerber
├── Mechanical/         # 结构工程 (SolidWorks)
│   └── Export/         # STL/DXF 加工文件
├── Firmware/           # 嵌入式代码 (Keil/VSCode)
│   ├── Core/           # CubeMX 生成代码
│   ├── Drivers/        # 硬件驱动
│   └── App/            # 核心业务逻辑
└── README.md           # 项目说明书
```
---

## 📅 开发计划 (Roadmap)

本项目采用 **T+Timeline** 管理，T 为核心物料到货日。

- [x] **T-3**: 方案冻结，BOM 采购，GitHub 环境搭建
- [ ] **T+0**: 物料到货，硬件/结构组启动
- [ ] **T+3**: 结构素组，电路焊接，软件基础框架完成
- [ ] **T+7**: **MVP 交付** (开环控制，蓝牙遥控跑通)
- [ ] **T+14**: PID 闭环算法迭代，失效保护功能
- [ ] **T+17**: 整车验收，技术文档汇总

---

## 👥 核心团队 (Team)

| 成员 | 角色 | 职责 |
| :--- | :--- | :--- |
| **@zhu-siyuan** | **PM & Software** | 项目管理、架构设计、STM32 固件开发、PID 调试 |
| **@min-huang-HW** | **Hardware** | 硬件选型、原理图/PCB 设计、电路焊接与测试 |
| **@yuying-qin-MD** | **Mechanical** | 工业设计、3D 建模 (SolidWorks)、结构总装 |

---

> Powered by **Micro-AGV Research Team**. 
> *Documentation is code.*