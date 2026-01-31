package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ShooterIOPheonix6 implements ShooterIO {
  private static TalonFX shooterMotor1, shooterMotor2, shooterMotor3, hoodMotor;
  private final VelocityTorqueCurrentFOC shooterVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage hoodMotionMagicRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut shooterVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

  public ShooterIOPheonix6() {
    CANBus kCANBus = new CANBus(Constants.MotorCANIds.CanBusName);
    shooterMotor1 = new TalonFX(Constants.MotorCANIds.shooterMotor1CANId, kCANBus);
    shooterMotor2 = new TalonFX(Constants.MotorCANIds.shooterMotor2CANId, kCANBus);
    shooterMotor3 = new TalonFX(Constants.MotorCANIds.shooterMotor3CANId, kCANBus);
    hoodMotor = new TalonFX(Constants.MotorCANIds.hoodMotorCANId, kCANBus);

    TalonFXConfiguration shooterConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.ShooterSubsystemPID.shooterKP)
                    .withKI(Constants.ShooterSubsystemPID.shooterKI)
                    .withKD(Constants.ShooterSubsystemPID.shooterKD)
                    .withKS(Constants.ShooterSubsystemPID.shooterKS)
                    .withKV(Constants.ShooterSubsystemPID.shooterKV));
    shooterMotor1.getConfigurator().apply(shooterConfig);
    shooterMotor2.getConfigurator().apply(shooterConfig);
    shooterMotor3.getConfigurator().apply(shooterConfig);

    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(
                new Slot0Configs()
                    .withKP(Constants.ShooterSubsystemPID.hoodKP)
                    .withKI(Constants.ShooterSubsystemPID.hoodKI)
                    .withKD(Constants.ShooterSubsystemPID.hoodKD)
                    .withKS(Constants.ShooterSubsystemPID.hoodKS)
                    .withKV(Constants.ShooterSubsystemPID.hoodKV))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        Constants.ShooterSubsystemPID.hoodMotionMagicCruiseVelocity)
                    .withMotionMagicAcceleration(
                        Constants.ShooterSubsystemPID.hoodMotionMagicAcceleration)
                    .withMotionMagicJerk(Constants.ShooterSubsystemPID.hoodMotionMagicJerk));
    hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void ShooterSetRps(double rps) {
    shooterMotor1.setControl(shooterVelocityRequest.withVelocity(rps));
    shooterMotor2.setControl(shooterVelocityRequest.withVelocity(rps));
    shooterMotor3.setControl(shooterVelocityRequest.withVelocity(rps));
  }

  @Override
  public void ShooterSetV(double voltage) {
    shooterMotor1.setControl(shooterVoltageRequest.withOutput(voltage));
    shooterMotor2.setControl(shooterVoltageRequest.withOutput(voltage));
    shooterMotor3.setControl(shooterVoltageRequest.withOutput(voltage));
  }

  @Override
  public void HoodSetAngle(double angle) {
    hoodMotor.setControl(hoodMotionMagicRequest.withPosition(angle));
  }

  @Override
  public void HoodSetZero() {
    hoodMotor.setPosition(0.0);
  }

  @Override
  public void HoodSetV(double voltage) {
    hoodMotor.setControl(hoodVoltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.ShooterRPS =
        (shooterMotor1.getVelocity().getValueAsDouble()
                + shooterMotor2.getVelocity().getValueAsDouble()
                + shooterMotor3.getVelocity().getValueAsDouble())
            / 3.0;
    inputs.ShooterCurrentAMPS =
        (shooterMotor1.getSupplyCurrent().getValueAsDouble()
                + shooterMotor2.getSupplyCurrent().getValueAsDouble()
                + shooterMotor3.getSupplyCurrent().getValueAsDouble())
            / 3.0;
    inputs.HoodAngle = hoodMotor.getPosition().getValueAsDouble();
    inputs.HoodVoltageV = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.HoodCurrentAMPS = hoodMotor.getSupplyCurrent().getValueAsDouble();
  }
}
