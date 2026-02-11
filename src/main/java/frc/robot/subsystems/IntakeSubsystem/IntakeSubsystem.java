package frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public static IntakeSubsystem m_Instance = null;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double lastLimit = -1.0;

  public static IntakeSubsystem getInstance() {
    return m_Instance == null ? m_Instance = new IntakeSubsystem() : m_Instance;
  }

  public IntakeSubsystem() {
    if (Robot.isReal()) {
      io = new IntakeIOPheonix6();
    } else {
      io = new IntakeIO() {};
    }
    io.pivotSetZero();
  }

  public void setIntakeRps(double rps) {
    io.intakesetrps(rps);
  }

  public void setIntakeVoltage(double voltage) {
    io.IntakesetV(voltage);
  }

  public void setPivotAngle(double degrees) {
    io.Pivotsetangle(degrees);
  }

  public void setPivotVoltage(double voltage) {
    io.pivotsetV(voltage);
  }

  public void setPivotZero() {
    io.pivotSetZero();
  }

  public void setSystemCurrentLimit(double amps) {
    if (Math.abs(amps - lastLimit) > 0.1) {
      io.setStatorCurrentLimit(amps);
      lastLimit = amps;
    }
  }

  public double getIntakeVelocityRps() {
    return inputs.intakeVelocityRPS;
  }

  public double getPivotAngleDegrees() {
    return inputs.PivotAngledegrees;
  }

  public double getIntakeCurrentAmps() {
    return inputs.intakeCurrentAMPS;
  }

  public double getPivotCurrentAmps() {
    return inputs.pivotCurrentAMPS;
  }

  public double getPivotVelocityRps() {
    return inputs.PivotVelocityRPS;
  }

  public void processLog() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void processDashboard() {
    SmartDashboard.putNumber("Intake/VelocityRPS", inputs.intakeVelocityRPS);
    SmartDashboard.putNumber("Intake/PivotAngleDegrees", inputs.PivotAngledegrees);
    SmartDashboard.putNumber("Intake/IntakeCurrentAMPS", inputs.intakeCurrentAMPS);
    SmartDashboard.putNumber("Intake/PivotCurrentAMPS", inputs.pivotCurrentAMPS);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }
}
