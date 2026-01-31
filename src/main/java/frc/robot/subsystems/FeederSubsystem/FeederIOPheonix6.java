package frc.robot.subsystems.FeederSubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeederIOPheonix6 implements FeederIO {
  private static TalonFX IndexerMotor, BeltMotor;
  private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut beltVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage indexerVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage beltVelocityRequest = new VelocityVoltage(0.0);

  public FeederIOPheonix6() {
    CANBus kCANBus = new CANBus(Constants.MotorCANIds.CanBusName);
    IndexerMotor = new TalonFX(Constants.MotorCANIds.indexerMotorCANId, kCANBus);
    BeltMotor = new TalonFX(Constants.MotorCANIds.beltMotorCANId, kCANBus);

    TalonFXConfiguration indexerConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.FeederSubsystemPID.indexerKP)
                    .withKI(Constants.FeederSubsystemPID.indexerKI)
                    .withKD(Constants.FeederSubsystemPID.indexerKD)
                    .withKS(Constants.FeederSubsystemPID.indexerKS)
                    .withKV(Constants.FeederSubsystemPID.indexerKV));
    IndexerMotor.getConfigurator().apply(indexerConfig);

    TalonFXConfiguration beltConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.FeederSubsystemPID.beltKP)
                    .withKI(Constants.FeederSubsystemPID.beltKI)
                    .withKD(Constants.FeederSubsystemPID.beltKD)
                    .withKS(Constants.FeederSubsystemPID.beltKS)
                    .withKV(Constants.FeederSubsystemPID.beltKV));
    BeltMotor.getConfigurator().apply(beltConfig);
  }

  @Override
  public void BeltSetRps(double RPS) {
    BeltMotor.setControl(beltVelocityRequest.withVelocity(RPS));
  }

  @Override
  public void IndexerSetRps(double RPS) {
    IndexerMotor.setControl(indexerVelocityRequest.withVelocity(RPS));
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
  }
}
