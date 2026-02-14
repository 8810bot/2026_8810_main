package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manual shooter command with mode-based control (same 6 modes as Aimbot). Target RPS comes from
 * Manual/ShooterRPS instead of distance interpolation. Hood is controlled manually via right stick.
 */
public class ManualShoot extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final DoubleSupplier rightStickY;

  private static final LoggedTunableNumber targetRPS =
      new LoggedTunableNumber("Manual/ShooterRPS", 60.0);
  private static final LoggedTunableNumber rpsMultiplier =
      new LoggedTunableNumber("Manual/ShooterRPSMultiplier", 1.05);

  private final Timer shotTimer = new Timer();
  private final Timer speedStableTimer = new Timer();
  private boolean isShooting = false;
  private double bangBangLastCurrent = 0.0;

  public ManualShoot(ShooterSubsystem shooterSubsystem, DoubleSupplier rightStickY) {
    this.shooterSubsystem = shooterSubsystem;
    this.rightStickY = rightStickY;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    isShooting = false;
    bangBangLastCurrent = 0.0;
    shotTimer.stop();
    shotTimer.reset();
    speedStableTimer.stop();
    speedStableTimer.reset();
  }

  @Override
  public void execute() {
    double finalShooterRPS = targetRPS.get() * rpsMultiplier.get();
    int controlMode = (int) ShooterSubsystem.shooterControlMode.get();

    // === Hood: Manual right stick voltage control ===
    double hoodVolts = -MathUtil.applyDeadband(rightStickY.getAsDouble(), 0.1) * 12.0;
    shooterSubsystem.setHoodVoltage(hoodVolts);

    // === Calculate Pulse Feedforward (Used in Mode 0 and Optional Mode 2/4) ===
    double pulseFeedforward = 0;
    double timeSinceStart = shotTimer.get() - ShooterSubsystem.shotDelay.get();
    if (timeSinceStart >= 0) {
      double period = ShooterSubsystem.shotFirePeriod.get();
      if (period > 0 && (timeSinceStart % period) < ShooterSubsystem.shotPulseDuration.get()) {
        pulseFeedforward = ShooterSubsystem.shotCurrentFF.get();
      }
    }

    // === Select Control Strategy Based on Stage and Mode ===
    boolean isStable = false;

    // === Mode 5: Pure Open Loop (Force Fire) ===
    if (controlMode == 5) {
      isShooting = true;
      isStable = true;
      double current = ShooterSubsystem.shooterOpenLoopCurrent.get();
      shooterSubsystem.setShooterCurrent(current);

      Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING_ALWAYS");
      Logger.recordOutput("ManualShoot/ShooterMode", "MODE5_PURE_OPENLOOP");
      Logger.recordOutput("ManualShoot/TotalCurrent", current);

    } else {
      // === Modes 0-4: Standard Logic with Spin Up ===

      // Speed stability check
      if (shooterSubsystem.isAtSetSpeed(finalShooterRPS)) {
        speedStableTimer.start();
      } else {
        speedStableTimer.stop();
        speedStableTimer.reset();
      }

      isStable = shooterSubsystem.isAtSetSpeed(finalShooterRPS) && speedStableTimer.get() > 0.1;

      // Mode 4: Once shooting, force stable to prevent oscillation
      if (controlMode == 4 && isShooting) {
        isStable = true;
      }

      if (isStable && !isShooting) {
        isShooting = true;
        shotTimer.start();
      }

      if (!isStable && !isShooting) {
        // ====== Stage 1: Spin Up (Same for all modes) ======
        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
        Logger.recordOutput("ManualShoot/ShooterStage", "SPIN_UP");
        Logger.recordOutput("ManualShoot/ShooterMode", "SLOT0_PID");

      } else if (isShooting) {
        // ====== Stage 2: Shooting (Branch based on mode) ======

        if (controlMode == 0) {
          // === Mode 0: Full PID + Pulse Feedforward ===
          shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, pulseFeedforward);
          Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING");
          Logger.recordOutput("ManualShoot/ShooterMode", "MODE0_PID_PULSE");
          Logger.recordOutput("ManualShoot/PulseFeedforward", pulseFeedforward);

        } else if (controlMode == 1) {
          // === Mode 1: Pure Open Loop Shooting ===
          double openLoopCurrent = ShooterSubsystem.shooterOpenLoopCurrent.get();
          shooterSubsystem.setShooterCurrent(openLoopCurrent);
          Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING");
          Logger.recordOutput("ManualShoot/ShooterMode", "MODE1_PURE_OPENLOOP");
          Logger.recordOutput("ManualShoot/OpenLoopCurrent", openLoopCurrent);

        } else if (controlMode == 2) {
          // === Mode 2: Weak PID + Open Loop Shooting (Optional Pulse Overlay) ===
          double feedforward = ShooterSubsystem.shooterOpenLoopCurrent.get();

          if (ShooterSubsystem.shooterMode2EnablePulse.get() > 0.5) {
            feedforward += pulseFeedforward;
          }

          shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 1, feedforward);
          Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING");
          Logger.recordOutput("ManualShoot/ShooterMode", "MODE2_HYBRID_CONTROL");
          Logger.recordOutput("ManualShoot/HybridFeedforward", feedforward);
          Logger.recordOutput("ManualShoot/PulseFeedforward", pulseFeedforward);

        } else if (controlMode == 3) {
          // === Mode 3: Bang-Bang Controller (With Deadband) ===
          double currentRPS = shooterSubsystem.getShooterRps();
          double deadband = ShooterSubsystem.bangBangDeadband.get();
          double error = finalShooterRPS - currentRPS;

          double bangBangOutput;
          if (error > deadband) {
            bangBangOutput = ShooterSubsystem.bangBangHighCurrent.get();
          } else if (error < -deadband) {
            bangBangOutput = ShooterSubsystem.bangBangLowCurrent.get();
          } else {
            bangBangOutput = ShooterSubsystem.bangBangSteadyCurrent.get();
          }

          bangBangLastCurrent = bangBangOutput;
          shooterSubsystem.setShooterCurrent(bangBangOutput);
          Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING");
          Logger.recordOutput("ManualShoot/ShooterMode", "MODE3_BANGBANG");
          Logger.recordOutput("ManualShoot/BangBangOutput", bangBangOutput);
          Logger.recordOutput("ManualShoot/BangBangError", error);

        } else if (controlMode == 4) {
          // === Mode 4: Enhanced Hybrid (RIO-Side Control) ===
          double baseCurrent = ShooterSubsystem.shooterOpenLoopCurrent.get();

          double pulseCurrent = 0.0;
          if (ShooterSubsystem.shooterMode2EnablePulse.get() > 0.5) {
            pulseCurrent = pulseFeedforward;
          }

          double currentRPS = shooterSubsystem.getShooterRps();
          double error = finalShooterRPS - currentRPS;
          double kP = ShooterSubsystem.shooterSlot1_kP.get();
          double pidOutput = error * kP;

          double totalCurrent = baseCurrent + pulseCurrent + pidOutput;

          shooterSubsystem.setShooterCurrent(totalCurrent);
          Logger.recordOutput("ManualShoot/ShooterStage", "SHOOTING");
          Logger.recordOutput("ManualShoot/ShooterMode", "MODE4_ENHANCED_HYBRID_RIO");
          Logger.recordOutput("ManualShoot/PulseFeedforward", pulseCurrent);
          Logger.recordOutput("ManualShoot/TotalFeedforward", baseCurrent + pulseCurrent);
          Logger.recordOutput("ManualShoot/TotalCurrent", totalCurrent);
          Logger.recordOutput("ManualShoot/PIDOutput", pidOutput);
        }

      } else {
        // ====== Stable but not yet shooting: Maintain spin up ======
        shooterSubsystem.setShooterRpsWithSlot(finalShooterRPS, 0, 0.0);
        Logger.recordOutput("ManualShoot/ShooterStage", "STABLE_IDLE");
        Logger.recordOutput("ManualShoot/ShooterMode", "SLOT0_PID");
      }
    }

    // === Common Logs ===
    Logger.recordOutput("ManualShoot/TargetRPS", targetRPS.get());
    Logger.recordOutput("ManualShoot/FinalShooterRPS", finalShooterRPS);
    Logger.recordOutput("ManualShoot/IsStable", isStable);
    Logger.recordOutput("ManualShoot/IsShooting", isShooting);
    Logger.recordOutput("ManualShoot/ControlModeSelect", controlMode);
    Logger.recordOutput("ManualShoot/TimeSinceStart", timeSinceStart);
    Logger.recordOutput("ManualShoot/CurrentRPS", shooterSubsystem.getShooterRps());
    Logger.recordOutput("ManualShoot/HoodVolts", hoodVolts);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterVoltage(0);
    shooterSubsystem.setHoodVoltage(0);
    isShooting = false;
    shotTimer.stop();
    shotTimer.reset();
    speedStableTimer.stop();
    speedStableTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
