# GEMINI.md

This file provides guidance to Antigravity (Gemini) when working with code in this repository.

## 项目概述
这是一个 FRC 2026 赛季机器人项目 (Team 8810)。
框架: **AdvantageKit** (日志/回放), **WPILib 2026**, **Phoenix 6** (CTRE Swerve), **PathPlanner**.
硬件: Kraken X60 (驱动/转向), Pigeon 2.0 (陀螺仪), CANcoder, Limelight (视觉).

## 常用命令
- **构建**: `./gradlew build`
- **部署**: `./gradlew deploy` (需要连接机器人网络)
- **仿真**: `./gradlew simulateJava` (桌面物理仿真)
- **测试**: `./gradlew test` (运行 JUnit 测试)
- **格式化**: `./gradlew spotlessApply` (构建时自动运行，Google Java Format)
- **回放**: `./gradlew replayWatch` (使用 AdvantageScope 回放日志)

## 架构设计
### AdvantageKit IO 模式
硬件交互通过 IO 接口抽象，以支持真实机器人、仿真和回放模式的无缝切换。
- **子系统位置**: `src/main/java/frc/robot/subsystems`
- **逻辑与硬件分离**:
  - 逻辑在子系统类中 (如 `Drive.java`)。
  - 硬件输入定义在 `*IO` 接口中 (如 `ModuleIO.java`)。
  - 具体实现: `*IOTalonFX` (真实), `*IOSim` (仿真), `*IO{}` (回放)。
- **模式切换**: `Constants.Mode` 决定使用哪种实现 (REAL, SIM, REPLAY)。

### 代码结构
- `src/main/java/frc/robot`:
  - `subsystems/`: 机器人子系统 (Drive 等)。
  - `commands/`: 基于命令的动作定义 (如 `AimandDrive`, `AutonTrench`)。
  - `generated/`: CTRE Tuner X 生成的文件 (`TunerConstants.java`)，**不要手动修改**。
  - `util/`: 工具类，包含 `LimelightHelpers` (视觉) 和 `PhoenixUtil`。
  - `BuildConstants.java`: 自动生成的构建元数据。

## 编码规范
- **格式化**: 严格执行 `spotless` 检查。
- **单位**: 内部计算统一使用 SI 单位 (米, 弧度, 秒)。
- **日志**: 所有输入必须通过 `Logger.processInputs()` 记录。
- **硬件配置**: ID 和参数定义在 `TunerConstants.java` 或 `Constants.java`。

## 关键文件
- `TunerConstants.java`: Swerve 模块配置 (ID, 偏移量, PID)。
- `RobotContainer.java`: 按钮绑定和自动阶段选择器设置。
- `Constants.java`: 全局常量和运行模式选择。
