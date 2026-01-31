# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个 FRC (FIRST Robotics Competition) 机器人项目，基于 **AdvantageKit** 框架的 Swerve 驱动模板。使用 WPILib 2026 和 Phoenix 6 (CTRE) 硬件。

## 常用命令

```bash
# 构建项目
./gradlew build

# 部署到机器人
./gradlew deploy

# 运行仿真
./gradlew simulateJava

# 运行测试
./gradlew test

# 代码格式化 (自动在编译时执行)
./gradlew spotlessApply

# 日志回放 (Replay mode)
./gradlew replayWatch
```

## 架构设计

### IO 层抽象模式 (AdvantageKit 核心概念)

项目采用 AdvantageKit 的 IO 层抽象设计，实现硬件与逻辑分离：

- **IO 接口** (`GyroIO`, `ModuleIO`): 定义硬件交互的抽象接口
- **IO 实现**:
  - `*IOTalonFX` / `*IOPigeon2`: 真实硬件实现
  - `*IOSim`: 物理仿真实现
  - 空实现 `{}`: 用于日志回放模式

### 运行模式 (`Constants.Mode`)

在 `Constants.java` 中通过 `simMode` 切换：
- `REAL`: 运行在真实机器人 (RoboRIO)
- `SIM`: 物理仿真模式
- `REPLAY`: 日志回放模式

### Swerve 驱动系统

```
Drive (主控制器)
├── Module[4] (FL, FR, BL, BR)
│   └── ModuleIO (硬件抽象)
├── GyroIO (陀螺仪抽象)
├── PhoenixOdometryThread (高频里程计)
└── SwerveDrivePoseEstimator (位姿估计)
```

### 硬件配置

- **TunerConstants.java**: 由 CTRE Tuner X 生成，包含所有 Swerve 模块的电机 ID、编码器偏移、PID 增益等
- **CAN 总线**: `mainCAN` (支持 CAN FD)
- **电机**: TalonFX (Kraken X60 FOC)
- **陀螺仪**: Pigeon 2
- **编码器**: CANcoder

## 依赖项

- WPILib 2026.1.1
- AdvantageKit 26.0.0
- Phoenix 6 (CTRE)
- PathPlannerLib
- Studica (NavX 支持)

## 代码规范

- 使用 Spotless + Google Java Format 自动格式化
- 编译时自动执行 `spotlessApply`
- Event 分支部署时自动创建 git commit
