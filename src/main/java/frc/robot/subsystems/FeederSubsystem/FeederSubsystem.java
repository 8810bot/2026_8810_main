package frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
  public static FeederSubsystem m_Instance = null;

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private double lastLimit = -1.0;

  public static FeederSubsystem getInstance() {
    return m_Instance == null ? m_Instance = new FeederSubsystem() : m_Instance;
  }

  public FeederSubsystem() {
    if (Robot.isReal()) {
      io = new FeederIOPheonix6();
    } else {
      io = new FeederIO() {};
    }
  }

  public void setBeltRps(double rps) {
    io.BeltSetRps(rps);
  }

  public void setIndexerRps(double rps) {
    io.IndexerSetRps(rps);
  }

  public void setBeltVoltage(double voltage) {
    io.BeltSetV(voltage);
  }

  public void setIndexerVoltage(double voltage) {
    io.IndexerSetV(voltage);
  }

  public void setSystemCurrentLimit(double amps) {
    if (Math.abs(amps - lastLimit) > 1.0) {
      io.setStatorCurrentLimit(amps);
      lastLimit = amps;
    }
  }

  public double getBeltVelocityRps() {
    return inputs.BeltVelocityRPS;
  }

  public double getIndexerVelocityRps() {
    return inputs.IndexerVelocityRPS;
  }

  public double getBeltCurrentAmps() {
    return inputs.BeltCurrentAMPS;
  }

  public double getIndexerCurrentAmps() {
    return inputs.IndexerCurrentAMPS;
  }

  public double getBeltVoltage() {
    return inputs.BeltVoltageV;
  }

  public double getIndexerVoltage() {
    return inputs.IndexerVoltageV;
  }

  public void processLog() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void processDashboard() {
    SmartDashboard.putNumber("Feeder/BeltVelocityRPS", inputs.BeltVelocityRPS);
    SmartDashboard.putNumber("Feeder/IndexerVelocityRPS", inputs.IndexerVelocityRPS);
    SmartDashboard.putNumber("Feeder/BeltCurrentAMPS", inputs.BeltCurrentAMPS);
    SmartDashboard.putNumber("Feeder/IndexerCurrentAMPS", inputs.IndexerCurrentAMPS);
    SmartDashboard.putNumber("Feeder/BeltVoltageV", inputs.BeltVoltageV);
    SmartDashboard.putNumber("Feeder/IndexerVoltageV", inputs.IndexerVoltageV);
    SmartDashboard.putNumber("Feeder/IndexerMechanismRPS", inputs.IndexerMechanismRPS);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }
}
