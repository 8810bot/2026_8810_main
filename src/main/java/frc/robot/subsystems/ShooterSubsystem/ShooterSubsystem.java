package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  public static ShooterSubsystem m_Instance = null;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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
    io.ShooterSetRps(rps);
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

  public void processDashboard() {
    SmartDashboard.putNumber("Shooter/RPS", inputs.ShooterRPS);
    SmartDashboard.putNumber("Shooter/CurrentAMPS", inputs.ShooterCurrentAMPS);
    SmartDashboard.putNumber("Shooter/HoodAngle", inputs.HoodAngle);
    SmartDashboard.putNumber("Shooter/HoodVoltageV", inputs.HoodVoltageV);
    SmartDashboard.putNumber("Shooter/HoodCurrentAMPS", inputs.HoodCurrentAMPS);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }
}
