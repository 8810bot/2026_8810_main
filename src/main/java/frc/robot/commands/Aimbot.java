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
  private final LoggedTunableNumber beltRPS =
      new LoggedTunableNumber("Aimbot/Targets/BeltRPS", 60.0);
  private static final LoggedTunableNumber shooterRPSMultiplier =
      new LoggedTunableNumber("Aimbot/ShooterRPSMultiplier", 1.05);
  private final Timer shotTimer = new Timer();
  private final Timer speedStableTimer = new Timer();

  // Bang-Bang controller state maintenance
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
      DoubleSupplier beltVolts) { // Retain parameter for compatibility
    this.drive = drive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.indexerVolts = indexerVolts;
    // this.beltVolts = beltVolts; // Internal no longer uses voltage control, switched to beltRPS
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
      feederSubsystem.setBeltRps(beltRPS.get());
      swing = true;
      shotTimer.start();
    } else {
      if (!aimpid.atSetpoint()) {
        isShooting = false;
      }

      shotTimer.stop();
      shotTimer.reset();
    }

    // === Calculate Pulse Feedforward (Used in Mode 0 and Optional Mode 2) ===
    double pulseFeedforward = 0;
    double timeSinceStart = shotTimer.get() - ShooterSubsystem.shotDelay.get();
    if (timeSinceStart >= 0) {
      double period = ShooterSubsystem.shotFirePeriod.get();
      if (period > 0 && (timeSinceStart % period) < ShooterSubsystem.shotPulseDuration.get()) {
        pulseFeedforward = ShooterSubsystem.shotCurrentFF.get();
      }
    }

    // === Select Control Strategy Based on Stage and Mode ===
    double finalShooterRPS = targetRPS * shooterRPSMultiplier.get();
    int controlMode = (int) ShooterSubsystem.shooterControlMode.get();
    boolean isStable = false;

    // === Mode 5: Pure Open Loop (Force Fire) ===
    if (controlMode == 5) {
      isShooting = true; // Always shooting
      isStable = true;
      double current = ShooterSubsystem.shooterOpenLoopCurrent.get();
      shooterSubsystem.setShooterCurrent(current);

      Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING_ALWAYS");
      Logger.recordOutput("Aimbot/ShooterMode", "MODE5_PURE_OPENLOOP");
      Logger.recordOutput("Aimbot/TotalCurrent", current);

    } else {
      // === Modes 0-4: Standard Logic with Spin Up ===
      isStable = shooterSubsystem.isAtSetSpeed(finalShooterRPS) && speedStableTimer.get() > 0.1;

      // Special handling for Mode 4: Once shooting, force stable to prevent oscillation
      if (controlMode == 4 && isShooting) {
        isStable = true;
      }

      if (!isStable) {
        // ====== Stage 1: Spin Up (Same for all modes) ======
        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
        Logger.recordOutput("Aimbot/ShooterStage", "SPIN_UP");
        Logger.recordOutput("Aimbot/ShooterMode", "SLOT0_PID");

      } else if (isShooting) {
        // ====== Stage 2: Shooting (Branch based on mode) ======

        if (controlMode == 0) {
          // === Mode 0: Full PID + Pulse Feedforward (Current Scheme) ===
          shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, pulseFeedforward);
          Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
          Logger.recordOutput("Aimbot/ShooterMode", "MODE0_PID_PULSE");
          Logger.recordOutput("Aimbot/PulseFeedforward", pulseFeedforward);

        } else if (controlMode == 1) {
          // === Mode 1: Pure Open Loop Shooting ===
          double openLoopCurrent = ShooterSubsystem.shooterOpenLoopCurrent.get();
          shooterSubsystem.setShooterCurrent(openLoopCurrent);
          Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
          Logger.recordOutput("Aimbot/ShooterMode", "MODE1_PURE_OPENLOOP");
          Logger.recordOutput("Aimbot/OpenLoopCurrent", openLoopCurrent);

        } else if (controlMode == 2) {
          // === Mode 2: Weak PID + Open Loop Shooting (Optional Pulse Overlay) ===
          double feedforward = ShooterSubsystem.shooterOpenLoopCurrent.get();

          // Optional: Overlay Pulse Feedforward
          if (ShooterSubsystem.shooterMode2EnablePulse.get() > 0.5) {
            feedforward += pulseFeedforward;
          }

          shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 1, feedforward);
          Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
          Logger.recordOutput("Aimbot/ShooterMode", "MODE2_HYBRID_CONTROL");
          Logger.recordOutput("Aimbot/HybridFeedforward", feedforward);
          Logger.recordOutput("Aimbot/PulseFeedforward", pulseFeedforward);

        } else if (controlMode == 3) {
          // === Mode 3: Bang-Bang Controller (With Deadband) ===
          double currentRPS = shooterSubsystem.getShooterRps();
          double deadband = ShooterSubsystem.bangBangDeadband.get();
          double error = finalShooterRPS - currentRPS;

          double bangBangOutput;
          if (error > deadband) {
            // Speed significantly low, full acceleration
            bangBangOutput = ShooterSubsystem.bangBangHighCurrent.get();
          } else if (error < -deadband) {
            // Speed significantly high, deceleration
            bangBangOutput = ShooterSubsystem.bangBangLowCurrent.get();
          } else {
            // Inside deadband, use steady current
            bangBangOutput = ShooterSubsystem.bangBangSteadyCurrent.get();
          }

          bangBangLastCurrent = bangBangOutput;
          shooterSubsystem.setShooterCurrent(bangBangOutput);
          Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
          Logger.recordOutput("Aimbot/ShooterMode", "MODE3_BANGBANG");
          Logger.recordOutput("Aimbot/BangBangOutput", bangBangOutput);
          Logger.recordOutput("Aimbot/BangBangError", error);
        } else if (controlMode == 4) {
          // === Mode 4: Enhanced Hybrid (RIO-Side Control) ===
          // 1. Base Current (Open Loop)
          double baseCurrent = ShooterSubsystem.shooterOpenLoopCurrent.get();

          // 2. Pulse Current (Optional)
          double pulseCurrent = 0.0;
          if (ShooterSubsystem.shooterMode2EnablePulse.get() > 0.5) {
            pulseCurrent = pulseFeedforward;
          }

          // 3. Weak PID Correction (P-Only)
          double currentRPS = shooterSubsystem.getShooterRps();
          double error = finalShooterRPS - currentRPS;
          double kP = ShooterSubsystem.shooterSlot1_kP.get();
          double pidOutput = error * kP;

          // 4. Sum Total Current
          double totalCurrent = baseCurrent + pulseCurrent + pidOutput;

          // Command Motor with Pure Current
          shooterSubsystem.setShooterCurrent(totalCurrent);

          Logger.recordOutput("Aimbot/ShooterStage", "SHOOTING");
          Logger.recordOutput("Aimbot/ShooterMode", "MODE4_ENHANCED_HYBRID_RIO");
          Logger.recordOutput("Aimbot/TotalFeedforward", baseCurrent + pulseCurrent);
          Logger.recordOutput("Aimbot/TotalCurrent", totalCurrent);
          Logger.recordOutput("Aimbot/PIDOutput", pidOutput);
        }

      } else {
        // ====== Stable but not shooting: Maintain spin up ======
        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
        Logger.recordOutput("Aimbot/ShooterStage", "STABLE_IDLE");
        Logger.recordOutput("Aimbot/ShooterMode", "SLOT0_PID");
      }
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
    feederSubsystem.setBeltRps(0);
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
