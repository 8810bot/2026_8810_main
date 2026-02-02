package frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private final DCMotorSim beltSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
          DCMotor.getKrakenX60(1),
          1.0);
  private final DCMotorSim indexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 1.0),
          DCMotor.getKrakenX60(1),
          1.0);

  private final PIDController beltController = new PIDController(0.1, 0.0, 0.0);
  private final PIDController indexerController = new PIDController(0.1, 0.0, 0.0);

  private boolean beltClosedLoop = false;
  private boolean indexerClosedLoop = false;

  private double beltAppliedVolts = 0.0;
  private double indexerAppliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Belt Logic
    if (beltClosedLoop) {
      double currentRps = beltSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
      beltAppliedVolts = beltController.calculate(currentRps);
      beltAppliedVolts = Math.max(-12.0, Math.min(12.0, beltAppliedVolts));
    }
    beltSim.setInputVoltage(beltAppliedVolts);
    beltSim.update(0.02);

    inputs.BeltVelocityRPS = beltSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    inputs.BeltCurrentAMPS = beltSim.getCurrentDrawAmps();
    inputs.BeltVoltageV = beltAppliedVolts;

    // Indexer Logic
    if (indexerClosedLoop) {
      double currentRps = indexerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
      indexerAppliedVolts = indexerController.calculate(currentRps);
      indexerAppliedVolts = Math.max(-12.0, Math.min(12.0, indexerAppliedVolts));
    }
    indexerSim.setInputVoltage(indexerAppliedVolts);
    indexerSim.update(0.02);

    inputs.IndexerVelocityRPS = indexerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    inputs.IndexerCurrentAMPS = indexerSim.getCurrentDrawAmps();
    inputs.IndexerVoltageV = indexerAppliedVolts;
  }

  @Override
  public void BeltSetRps(double rps) {
    beltClosedLoop = true;
    beltController.setSetpoint(rps);
  }

  @Override
  public void IndexerSetRps(double rps) {
    indexerClosedLoop = true;
    indexerController.setSetpoint(rps);
  }

  @Override
  public void BeltSetV(double voltage) {
    beltClosedLoop = false;
    beltAppliedVolts = Math.max(-12.0, Math.min(12.0, voltage));
  }

  @Override
  public void IndexerSetV(double voltage) {
    indexerClosedLoop = false;
    indexerAppliedVolts = Math.max(-12.0, Math.min(12.0, voltage));
  }
}
