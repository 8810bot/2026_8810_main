package frc.robot.subsystems.FeederSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeederIOPheonix6 implements FeederIO {
  private static TalonFX IndexerMotor, BeltMotor;
  private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut beltVoltageRequest = new VoltageOut(0.0);
  private final VelocityTorqueCurrentFOC indexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC beltVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  public FeederIOPheonix6() {
    IndexerMotor =
        new TalonFX(Constants.MotorCANIds.indexerMotorCANId, Constants.MotorCANIds.CanBusName);
    BeltMotor = new TalonFX(Constants.MotorCANIds.beltMotorCANId, Constants.MotorCANIds.CanBusName);

    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.Slot0 = new Slot0Configs();
    indexerConfig.Slot0.kP = Constants.FeederSubsystemPID.indexerKP;
    indexerConfig.Slot0.kI = Constants.FeederSubsystemPID.indexerKI;
    indexerConfig.Slot0.kD = Constants.FeederSubsystemPID.indexerKD;
    indexerConfig.Slot0.kS = Constants.FeederSubsystemPID.indexerKS;
    indexerConfig.Slot0.kV = Constants.FeederSubsystemPID.indexerKV;
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    IndexerMotor.getConfigurator().apply(indexerConfig);

    TalonFXConfiguration beltConfig = new TalonFXConfiguration();
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    beltConfig.Slot0 = new Slot0Configs();
    beltConfig.Slot0.kP = Constants.FeederSubsystemPID.beltKP;
    beltConfig.Slot0.kI = Constants.FeederSubsystemPID.beltKI;
    beltConfig.Slot0.kD = Constants.FeederSubsystemPID.beltKD;
    beltConfig.Slot0.kS = Constants.FeederSubsystemPID.beltKS;
    beltConfig.Slot0.kV = Constants.FeederSubsystemPID.beltKV;
    beltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    BeltMotor.getConfigurator().apply(beltConfig);
  }

  @Override
  public void BeltSetRps(double RPS) {
    BeltMotor.setControl(beltVelocityRequest.withVelocity(RPS));
  }

  @Override
  public void IndexerSetRps(double RPS) {
    IndexerMotor.setControl(indexerVelocityRequest.withVelocity(RPS * 12 / 36));
  }

  @Override
  public void BeltSetV(double voltage) {
    BeltMotor.setControl(beltVoltageRequest.withOutput(voltage));
  }

  @Override
  public void IndexerSetV(double voltage) {
    IndexerMotor.setControl(indexerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.BeltVelocityRPS = BeltMotor.getVelocity().getValueAsDouble();
    inputs.IndexerVelocityRPS = IndexerMotor.getVelocity().getValueAsDouble();
    inputs.BeltCurrentAMPS = BeltMotor.getSupplyCurrent().getValueAsDouble();
    inputs.IndexerCurrentAMPS = IndexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.BeltVoltageV = BeltMotor.getMotorVoltage().getValueAsDouble();
    inputs.IndexerVoltageV = IndexerMotor.getMotorVoltage().getValueAsDouble();
    // Convert back to mechanism RPS: Motor RPS * (36/12)
    inputs.IndexerMechanismRPS = inputs.IndexerVelocityRPS * (36.0 / 12.0);
  }

  @Override
  public void setStatorCurrentLimit(double amps) {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimit = amps;
    config.StatorCurrentLimitEnable = true;

    IndexerMotor.getConfigurator().apply(config);
    BeltMotor.getConfigurator().apply(config);
  }
}
