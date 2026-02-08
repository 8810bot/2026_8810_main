package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem m_Instance = null;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", Constants.ShooterSubsystemPID.shooterKP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", Constants.ShooterSubsystemPID.shooterKI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", Constants.ShooterSubsystemPID.shooterKD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/kS", Constants.ShooterSubsystemPID.shooterKS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/kV", Constants.ShooterSubsystemPID.shooterKV);
  private static final LoggedTunableNumber shooterPeakReverseTorque =
      new LoggedTunableNumber(
          "Shooter/PeakReverseTorque",
          Constants.ShooterSubsystemPID.shooterPeakReverseTorqueCurrent);
  // === 控制模式选择 ===
  public static final LoggedTunableNumber shooterControlMode =
      new LoggedTunableNumber("Shooter/ControlMode", 0.0);
  // 0 = 全程PID+脉冲前馈（默认）, 1 = 纯开环射击, 2 = 弱PID+开环射击, 3 = Bang-Bang控制

  // === 开环电流参数（模式1、2使用）===
  public static final LoggedTunableNumber shooterOpenLoopCurrent =
      new LoggedTunableNumber("Shooter/OpenLoopCurrent", 40.0);

  // === Bang-Bang 参数（仅模式3使用）===
  public static final LoggedTunableNumber bangBangHighCurrent =
      new LoggedTunableNumber("Shooter/BangBangHighCurrent", 50.0);
  public static final LoggedTunableNumber bangBangLowCurrent =
      new LoggedTunableNumber("Shooter/BangBangLowCurrent", 35.0);
  public static final LoggedTunableNumber bangBangSteadyCurrent =
      new LoggedTunableNumber("Shooter/BangBangSteadyCurrent", 38.0);
  public static final LoggedTunableNumber bangBangDeadband =
      new LoggedTunableNumber("Shooter/BangBangDeadband", 1.0);

  // === Slot1 弱PID参数（仅模式2使用）===
  public static final LoggedTunableNumber shooterSlot1_kP =
      new LoggedTunableNumber("Shooter/Slot1_kP", 0.5);
  public static final LoggedTunableNumber shooterSlot1_kI =
      new LoggedTunableNumber("Shooter/Slot1_kI", 0.0);
  public static final LoggedTunableNumber shooterSlot1_kD =
      new LoggedTunableNumber("Shooter/Slot1_kD", 0.01);
  public static final LoggedTunableNumber shooterSlot1_kS =
      new LoggedTunableNumber("Shooter/Slot1_kS", 0.0);
  public static final LoggedTunableNumber shooterSlot1_kV =
      new LoggedTunableNumber("Shooter/Slot1_kV", 0.01);

  // === 脉冲前馈参数（模式0使用，模式2可选）===
  public static final LoggedTunableNumber shotCurrentFF =
      new LoggedTunableNumber("Shooter/ShotCurrentFF", 37.0);
  public static final LoggedTunableNumber shotDelay =
      new LoggedTunableNumber("Shooter/ShotDelay", 0.05);
  public static final LoggedTunableNumber shotPulseDuration =
      new LoggedTunableNumber("Shooter/ShotPulseDuration", 0.15);
  public static final LoggedTunableNumber shotFirePeriod =
      new LoggedTunableNumber("Shooter/ShotFirePeriod", 0.15);

  // === 模式2是否启用脉冲前馈 ===
  public static final LoggedTunableNumber shooterMode2EnablePulse =
      new LoggedTunableNumber("Shooter/Mode2EnablePulse", 0.0);

  public static ShooterSubsystem getInstance() {
    return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
  }

  public ShooterSubsystem() {
    if (Robot.isReal()) {
      io = new ShooterIOPheonix6();
    } else {
      io = new ShooterIO() {};
    }
    io.HoodSetZero();
  }

  public void setShooterRps(double rps) {
    setShooterRps(rps, 0.0);
  }

  public void setShooterRps(double rps, double feedforwardAmps) {
    io.ShooterSetRps(rps, feedforwardAmps);
  }

  public void setShooterRpsWithSlot(double rps, int slot, double feedforwardAmps) {
    io.ShooterSetRpsWithSlot(rps, slot, feedforwardAmps);
  }

  public void setShooterCurrent(double amps) {
    io.ShooterSetCurrent(amps);
  }

  public void setShooterVoltage(double voltage) {
    io.ShooterSetV(voltage);
  }

  public void setHoodAngle(double angle) {
    io.HoodSetAngle(angle / 360);
  }

  public void setHoodZero() {
    io.HoodSetZero();
  }

  public void setHoodVoltage(double voltage) {
    io.HoodSetV(voltage);
  }

  public double getShooterRps() {
    return inputs.ShooterRPS;
  }

  public double getShooterCurrentAmps() {
    return inputs.ShooterCurrentAMPS;
  }

  public double getHoodAngle() {
    return inputs.HoodAngle;
  }

  public double getHoodVoltage() {
    return inputs.HoodVoltageV;
  }

  public double getHoodCurrentAmps() {
    return inputs.HoodCurrentAMPS;
  }

  public void processLog() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Command to home the hood against the hardstop using current detection. */
  public Command runHoodHoming() {
    return Commands.sequence(
        // Step 1: Apply homing voltage (Wait 0.2s to ignore inrush current)
        Commands.run(
                () ->
                    io.HoodSetV(
                        frc.robot.Constants.ShooterSubsystemPID.HoodHomingConstants.kHomingVolts),
                this)
            .beforeStarting(Commands.waitSeconds(0.2))
            .until(
                () ->
                    Math.abs(inputs.HoodCurrentAMPS)
                        > frc.robot.Constants.ShooterSubsystemPID.HoodHomingConstants
                            .kHomingCurrentThresholdAmps),

        // Step 2: Stop and Zero
        Commands.runOnce(
            () -> {
              io.HoodSetV(0);
              io.HoodSetZero();
            },
            this));
  }

  public boolean isAtSetSpeed(double targetRPS) {
    return Math.abs(inputs.ShooterRPS - targetRPS) < 1.0;
  }

  public void processDashboard() {
    SmartDashboard.putNumber("Shooter/RPS", inputs.ShooterRPS);
    SmartDashboard.putNumber("Shooter/CurrentAMPS", inputs.ShooterCurrentAMPS);
    SmartDashboard.putNumber("Shooter/TorqueCurrentAMPS", inputs.ShooterTorqueCurrent);
    SmartDashboard.putNumber("Shooter/HoodAngle", inputs.HoodAngle);
    SmartDashboard.putNumber("Shooter/HoodVoltageV", inputs.HoodVoltageV);
    SmartDashboard.putNumber("Shooter/HoodCurrentAMPS", inputs.HoodCurrentAMPS);
  }

  @Override
  public void periodic() {
    // Check if Slot0 PID constants have changed
    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }

    // Check if Slot1 PID constants have changed
    if (shooterSlot1_kP.hasChanged(hashCode())
        || shooterSlot1_kI.hasChanged(hashCode())
        || shooterSlot1_kD.hasChanged(hashCode())
        || shooterSlot1_kS.hasChanged(hashCode())
        || shooterSlot1_kV.hasChanged(hashCode())) {
      io.setPIDSlot1(
          shooterSlot1_kP.get(),
          shooterSlot1_kI.get(),
          shooterSlot1_kD.get(),
          shooterSlot1_kS.get(),
          shooterSlot1_kV.get());
    }

    if (shooterPeakReverseTorque.hasChanged(hashCode())) {
      io.setPeakReverseTorque(shooterPeakReverseTorque.get());
    }

    processLog();
    processDashboard();
  }
}
