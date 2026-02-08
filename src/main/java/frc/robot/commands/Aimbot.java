package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Aimbot extends Command {
  private Drive drive;
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private double dist;
  private PIDController aimpid = new PIDController(5, 0, 0);
  private boolean swing = false;
  private boolean isShooting = false;
  private DoubleSupplier indexerVolts;
  private final LoggedTunableNumber indexerRPS =
      new LoggedTunableNumber("Aimbot/Targets/IndexerRPS", 70.0);
  private static final LoggedTunableNumber shooterRPSMultiplier =
      new LoggedTunableNumber("Aimbot/ShooterRPSMultiplier", 1.05);
  private DoubleSupplier beltVolts;
  private final Timer shotTimer = new Timer();
  private final Timer speedStableTimer = new Timer();

  // Bang-Bang 控制器状态保持
  private double bangBangLastCurrent = 0.0;

  private enum STAT {
    At_dgr1,
    moving,
    At_dgr2
  };

  private STAT state = STAT.At_dgr1;

  private IntakeSubsystem intakeSubsystem;

  public Aimbot(
      Drive drive,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsystem intakeSubsystem,
      DoubleSupplier indexerVolts,
      DoubleSupplier beltVolts) {
    this.drive = drive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.indexerVolts = indexerVolts;
    this.beltVolts = beltVolts;
    addRequirements(drive);
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);

    aimpid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    aimpid.setTolerance(0.087);
    dist = Drive.distanceToGoal;
    isShooting = false;
    shotTimer.stop();
    shotTimer.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double robotX = pose.getX();
    double robotY = pose.getY();
    double dx, dy = 0;

    // Vector from robot to goal
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      dx = Constants.aimconstants.bluegoalpos.getX() - robotX;
      dy = Constants.aimconstants.bluegoalpos.getY() - robotY;
    } else {
      dx = Constants.aimconstants.redgoalpos.getX() - robotX;
      dy = Constants.aimconstants.redgoalpos.getY() - robotY;
    }
    // Target angle to goal (field frame)
    double targetAngle = Math.atan2(dy, dx); // radians

    double turnOutput = aimpid.calculate(drive.getRotation().getRadians(), targetAngle);
    // Dynamic distance update
    dist = Drive.distanceToGoal;

    // Calculate targets based on distance (Calculate FIRST)
    double targetRPS = (double) Constants.aimconstants.distanceToShooterRPS.get(dist);
    double targetHoodAngle = (double) Constants.aimconstants.distanceToHoodAngle.get(dist);

    if (aimpid.atSetpoint()) {
      turnOutput = 0;
      drive.stopWithX();

      // Speed Stability Check
      if (shooterSubsystem.isAtSetSpeed(targetRPS * shooterRPSMultiplier.get())) {
        speedStableTimer.start();
      } else {
        speedStableTimer.stop();
        speedStableTimer.reset();
      }

      if (speedStableTimer.get() > 0.1) {
        isShooting = true;
      }
    } else {
      // Not at setpoint
      speedStableTimer.stop();
      speedStableTimer.reset();
    }

    if (isShooting) {
      // feederSubsystem.setIndexerVoltage(indexerVolts.getAsDouble());
      feederSubsystem.setIndexerRps(indexerRPS.get());
      feederSubsystem.setBeltVoltage(beltVolts.getAsDouble());
      swing = true;
      shotTimer.start();
    } else {
      if (!aimpid.atSetpoint()) {
        isShooting = false;
      }

      shotTimer.stop();
      shotTimer.reset();
    }

    // === 计算脉冲前馈（模式0和可选模式2使用）===
    double pulseFeedforward = 0;
    double timeSinceStart = shotTimer.get() - ShooterSubsystem.shotDelay.get();
    if (timeSinceStart >= 0) {
      double period = ShooterSubsystem.shotFirePeriod.get();
      if (period > 0 && (timeSinceStart % period) < ShooterSubsystem.shotPulseDuration.get()) {
        pulseFeedforward = ShooterSubsystem.shotCurrentFF.get();
      }
    }

    // === 根据阶段和模式选择控制策略 ===
    double finalShooterRPS = targetRPS * shooterRPSMultiplier.get();
    int controlMode = (int) ShooterSubsystem.shooterControlMode.get();
    boolean isStable =
        shooterSubsystem.isAtSetSpeed(finalShooterRPS) && speedStableTimer.get() > 0.1;

    if (!isStable) {
      // ====== 阶段1：稳速阶段（所有模式相同）======
      shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
      Logger.recordOutput("Aimbot/ShooterStage", "SPIN_UP");
      Logger.recordOutput("Aimbot/ShooterMode", "SLOT0_PID");

    } else if (isShooting) {
      // ====== 阶段2：射击阶段（根据模式分支）======

      if (controlMode == 0) {
        // === 模式0：全程PID + 脉冲前馈（当前方案）===
        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, pulseFeedforward);
        Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
        Logger.recordOutput("Aimbot/ShooterMode", "MODE0_PID_PULSE");
        Logger.recordOutput("Aimbot/PulseFeedforward", pulseFeedforward);

      } else if (controlMode == 1) {
        // === 模式1：纯开环射击 ===
        double openLoopCurrent = ShooterSubsystem.shooterOpenLoopCurrent.get();
        shooterSubsystem.setShooterCurrent(openLoopCurrent);
        Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
        Logger.recordOutput("Aimbot/ShooterMode", "MODE1_PURE_OPENLOOP");
        Logger.recordOutput("Aimbot/OpenLoopCurrent", openLoopCurrent);

      } else if (controlMode == 2) {
        // === 模式2：弱PID + 开环射击（可选叠加脉冲）===
        double feedforward = ShooterSubsystem.shooterOpenLoopCurrent.get();

        // 可选：叠加脉冲前馈
        if (ShooterSubsystem.shooterMode2EnablePulse.get() > 0.5) {
          feedforward += pulseFeedforward;
        }

        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 1, feedforward);
        Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
        Logger.recordOutput("Aimbot/ShooterMode", "MODE2_HYBRID_CONTROL");
        Logger.recordOutput("Aimbot/HybridFeedforward", feedforward);
        Logger.recordOutput("Aimbot/PulseFeedforward", pulseFeedforward);

      } else if (controlMode == 3) {
        // === 模式3：Bang-Bang 控制器（带死区）===
        double currentRPS = shooterSubsystem.getShooterRps();
        double deadband = ShooterSubsystem.bangBangDeadband.get();
        double error = finalShooterRPS - currentRPS;

        double bangBangOutput;
        if (error > deadband) {
          // 转速显著偏低，全力加速
          bangBangOutput = ShooterSubsystem.bangBangHighCurrent.get();
        } else if (error < -deadband) {
          // 转速显著偏高，减速
          bangBangOutput = ShooterSubsystem.bangBangLowCurrent.get();
        } else {
          // 在死区内，使用稳态电流
          bangBangOutput = ShooterSubsystem.bangBangSteadyCurrent.get();
        }

        bangBangLastCurrent = bangBangOutput;
        shooterSubsystem.setShooterCurrent(bangBangOutput);
        Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
        Logger.recordOutput("Aimbot/ShooterMode", "MODE3_BANGBANG");
        Logger.recordOutput("Aimbot/BangBangOutput", bangBangOutput);
        Logger.recordOutput("Aimbot/BangBangError", error);
      }

    } else {
      // ====== 稳定但未射击：保持稳速状态 ======
      shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
      Logger.recordOutput("Aimbot/ShooterStage", "STABLE_IDLE");
      Logger.recordOutput("Aimbot/ShooterMode", "SLOT0_PID");
    }

    shooterSubsystem.setHoodAngle(targetHoodAngle);

    // === 通用日志输出 ===
    Logger.recordOutput("Aimbot/FinalShooterRPS", finalShooterRPS);
    Logger.recordOutput("Aimbot/IsStable", isStable);
    Logger.recordOutput("Aimbot/IsShooting", isShooting);
    Logger.recordOutput("Aimbot/ControlModeSelect", controlMode);
    Logger.recordOutput("Aimbot/TimeSinceStart", timeSinceStart);

    if (swing) {
      double current_dgr = intakeSubsystem.getPivotAngleDegrees();
      if (Constants.ShooterSubsystemPID.swing_angle_1 + 2 < current_dgr && state == STAT.At_dgr1
          || Constants.ShooterSubsystemPID.swing_angle_2 - 2 > current_dgr
              && state == STAT.At_dgr2) {
        state = STAT.moving;
      } else if (Math.abs(Constants.ShooterSubsystemPID.swing_angle_2 - current_dgr) <= 2
          && state == STAT.moving) {
        state = STAT.At_dgr2;
      } else if (Math.abs(Constants.ShooterSubsystemPID.swing_angle_1 - current_dgr) <= 2
          && state == STAT.moving) {
        state = STAT.At_dgr1;
      }
      switch (state) {
        case At_dgr1:
          intakeSubsystem.setPivotAngle(Constants.ShooterSubsystemPID.swing_angle_2);
          break;
        case At_dgr2:
          intakeSubsystem.setPivotAngle(Constants.ShooterSubsystemPID.swing_angle_1);
          break;
      }
    }

    Logger.recordOutput("Aimbot/TargetRPS", targetRPS);
    Logger.recordOutput("Aimbot/TargetHoodAngle", targetHoodAngle);
    Logger.recordOutput("Aimbot/Distance", dist);
    Logger.recordOutput("Aimbot/AtSetpoint", aimpid.atSetpoint());
    Logger.recordOutput("Aimbot/IsShooting", isShooting);
    Logger.recordOutput("Aimbot/SpeedStableTimer", speedStableTimer.get());
    Logger.recordOutput("Aimbot/ShotTimer", shotTimer.get());
    Logger.recordOutput("Aimbot/PIDError", aimpid.getPositionError());
    Logger.recordOutput("Aimbot/TargetYaw", targetAngle);
    if (DriverStation.getAlliance().isPresent()) {
      Logger.recordOutput(
          "Aimbot/TargetPosition",
          new Pose2d(
              DriverStation.getAlliance().get() == Alliance.Blue
                  ? Constants.aimconstants.bluegoalpos
                  : Constants.aimconstants.redgoalpos,
              new edu.wpi.first.math.geometry.Rotation2d())); // Log target position
    }

    ChassisSpeeds aimspeed = new ChassisSpeeds(0, 0, turnOutput);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIndexerRps(0);
    // feederSubsystem.setIndexerVoltage(0);
    shooterSubsystem.setShooterVoltage(0);
    feederSubsystem.setBeltVoltage(0);
    intakeSubsystem.setPivotAngle(0);
    isShooting = false;
    shotTimer.stop();
    shotTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
