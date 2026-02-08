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
  public static final LoggedTunableNumber shotCurrentFF =
      new LoggedTunableNumber("Shooter/ShotCurrentFF", 37.0);
  public static final LoggedTunableNumber shotDelay =
      new LoggedTunableNumber("Shooter/ShotDelay", 0.05);
  public static final LoggedTunableNumber shotPulseDuration =
      new LoggedTunableNumber("Shooter/ShotPulseDuration", 0.15);
  public static final LoggedTunableNumber shotFirePeriod =
      new LoggedTunableNumber("Shooter/ShotFirePeriod", 0.15);

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
    // Check if PID constants have changed
    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }
    if (shooterPeakReverseTorque.hasChanged(hashCode())) {
      io.setPeakReverseTorque(shooterPeakReverseTorque.get());
    }

    processLog();
    processDashboard();
  }
}
