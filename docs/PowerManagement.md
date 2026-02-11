# 整车功率分配控制系统方案

## 1. 背景与需求

### 1.1 问题现状
- 射击系统（Shooter）在全速射击时需要约 200A 电流。
- 底盘系统（Drive）全速驱动峰值电流可达 480A (120A × 4)。
- 现有系统缺乏全局功率管理，若两者同时全速运行，总电流可能超过 700A，导致电池电压瞬间跌落至 6.8V 以下（RoboRIO Brownout），引发系统重启或通讯丢失。
- 目前各子系统独立运行，Shooter/Feeder/Intake 等电机均无硬件级电流限制，存在烧毁风险。

### 1.2 目标
- **核心目标**：确保射击阶段 Shooter 获得足额功率（200A+），防止因底盘抢占功率导致射速波动。
- **次要目标**：防止系统电压过低导致 Brownout。
- **要求**：
  - 必须遵循 AdvantageKit IO 分层架构。
  - 必须有明确的 Dashboard 指示，严禁静默降级。
  - 具备高扩展性，易于添加新子系统。

## 2. 系统架构设计 (AdvantageKit 风格)

系统采用 **"策略-执行" 分离** 的架构，严格遵循 AdvantageKit 的单向数据流和 IO 隔离原则。

```mermaid
graph TD
    PM[PowerManager (Subsystem)] -->|1. 读取 Inputs| PMIO[PowerManagerIO (PDH)]
    PM -->|2. 计算策略| State[LoggedPowerState]
    RC[RobotContainer] -->|3. 读取策略状态| PM
    RC -->|4. 分发限流指令| Drive[Drive Subsystem]
    RC -->|4. 分发限流指令| Shooter[Shooter Subsystem]
    Drive -->|5. 更新 IO Inputs| DriveIO
    Shooter -->|5. 更新 IO Inputs| ShooterIO
```

### 2.1 核心组件

1.  **PowerManager (Subsystem)**:
    - **职责**: 纯粹的策略计算中心。不直接控制电机，不持有其他 Subsystem 引用。
    - **输入**: PDH 电压/电流 (通过 `PowerManagerIO`)。
    - **状态**: 维护当前 `PowerProfile` 和 `VoltageState`。
    - **输出**: 公开 `getCurrentState()` 方法，返回包含各子系统限流值的对象。

2.  **Subsystems (Drive, Shooter, etc.)**:
    - **职责**: 执行具体的限流操作。
    - **接口**: 新增 `setSystemCurrentLimit(double amps)` 方法。
    - **IO层**: `*IO` 接口新增 `setStatorCurrentLimit(double amps)`，在 `*IOPheonix6` 中调用硬件 API。

3.  **RobotContainer**:
    - **职责**: 系统的 "胶水"。
    - **绑定**: 在 `robotPeriodic` 或通过 `Command` 将 PowerManager 的计算结果传递给各子系统。

## 3. 详细设计

### 3.1 功率配置 (PowerProfile)

系统预定义以下几种典型工况（Profile），每种工况对应一张**静态电流预算表**（单位：Amps, Stator Current）：

| Profile | 描述 | Drive (4x) | Shooter (3x) | Feeder (2x) | Intake (2x) | Hood (1x) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **IDLE** | 待机 | 40A | 10A | 10A | 10A | 10A |
| **DRIVING** | 纯移动 | **120A** | 10A | 10A | 10A | 10A |
| **SHOOTING** | 静止射击 | 40A | **100A** | 40A | 20A | 20A |
| **SHOOT_AND_DRIVE** | 移动射击 | 80A | 80A | 30A | 15A | 15A |
| **AUTO** | 自动阶段 | 100A | 90A | 35A | 15A | 15A |

### 3.2 动态降功率算法 (Dynamic Throttling)

**控制逻辑 (20ms 周期执行)**：
1.  读取 PDH 总电压 $V_{batt}$。
2.  计算 **Throttle Factor (限制系数)** $\alpha$：
    - 若 $V_{batt} > V_{warn}$ (10.5V): $\alpha = 1.0$ (不限制)
    - 若 $V_{batt} < V_{danger}$ (8.0V): $\alpha = 0.0$ (最大限制)
    - 中间区域线性插值：$\alpha = \frac{V_{batt} - V_{danger}}{V_{warn} - V_{danger}}$
3.  计算最终限流值 $I_{limit}$：
    $$I_{limit} = I_{min} + (I_{profile} - I_{min}) \times \alpha$$

### 3.3 接口定义

#### `PowerManager` Subsystem
```java
public class PowerManager extends SubsystemBase {
    // 内部类：包含所有子系统的限流推荐值
    public static class PowerDistributionState {
        public double driveCurrentLimit = 40.0;
        public double shooterCurrentLimit = 10.0;
        public double feederCurrentLimit = 10.0;
        public double intakeCurrentLimit = 10.0;
    }

    /** 获取当前的功率分配状态 */
    public PowerDistributionState getCurrentState();

    /** 设置当前的工况 */
    public void setProfile(PowerProfile profile);
}
```

#### 各 Subsystem (以 Shooter 为例)
```java
public class ShooterSubsystem extends SubsystemBase {
    /** 设置来自电源管理系统的限流值 */
    public void setSystemCurrentLimit(double amps) {
        // 避免重复调用硬件API：仅当值变化超过阈值(如 1A)时才下发
        if (Math.abs(amps - lastLimit) > 1.0) {
             io.setStatorCurrentLimit(amps);
             lastLimit = amps;
        }
    }
}
```

### 3.4 集成方式

1.  **RobotContainer**:
    - 在 `configureButtonBindings` 中绑定 Profile 切换：
      ```java
      // 按住射击键：切换到 SHOOTING，松开回 DRIVING
      controller.rightTrigger().whileTrue(
          Commands.runOnce(() -> powerManager.setProfile(PowerProfile.SHOOTING))
          .andThen(new Aimbot(...))
          .finallyDo(() -> powerManager.setProfile(PowerProfile.DRIVING))
      );
      ```
    - **关键：闭环分发**
      使用 `Commands.run()` 创建一个后台命令（或在 `robotPeriodic`），持续将 PowerManager 的状态同步给子系统：
      ```java
      // 在构造函数中
      powerManager.setDefaultCommand(Commands.run(() -> {
          var state = powerManager.getCurrentState();
          drive.setSystemCurrentLimit(state.driveCurrentLimit);
          shooterSubsystem.setSystemCurrentLimit(state.shooterCurrentLimit);
          feederSubsystem.setSystemCurrentLimit(state.feederCurrentLimit);
          intakeSubsystem.setSystemCurrentLimit(state.intakeCurrentLimit);
      }, powerManager));
      ```
      *注：这样 PowerManager 不需要知道其他 Subsystem 的存在，由 RobotContainer 负责 "连线" 。*

## 4. 监控与日志 (AdvantageKit)

所有决策过程必须透明可查，记录到 `PowerManager/` 下：

- `State`: 当前状态 (NORMAL / WARNING / DANGER)
- `Profile`: 当前工况 (DRIVING / SHOOTING...)
- `BatteryVoltage`: 电池电压
- `ThrottleFactor`: 动态限制系数 (0.0 - 1.0)
- `Allocations/<Subsystem>`: 分配给各子系统的电流限额

## 5. 安全特性

1.  **失效保护**: 若 PDH 读数失效，默认维持最后一次有效的 Profile 限流，不进行动态降功率。
2.  **通信保护**: CAN 总线繁忙时，限流指令发送频率限制为 10Hz（如无变化则不发送）。
3.  **Brownout 恢复**: 电压回升后，设置滞后（Hysteresis）以防止限制系数震荡。

## 6. 开发计划

1.  创建 `PowerManager` 子系统及相关类。
2.  修改 `Drive`, `Shooter` 等子系统实现 `PowerConsumer` 接口。
3.  在 `RobotContainer` 中集成并绑定 Profile 切换。
4.  在仿真环境验证 Profile 切换逻辑。
5.  实机测试，调整 $V_{warn}$ 和 $V_{danger}$ 阈值。
