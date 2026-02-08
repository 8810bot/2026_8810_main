package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemPID;

public class ShooterIOPheonix6 implements ShooterIO {
  private static TalonFX shooterMotor1, shooterMotor2, shooterMotor3, hoodMotor;
  private final VelocityTorqueCurrentFOC shooterVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC shooterCurrentRequest = new TorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC hoodMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final VoltageOut shooterVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

  public ShooterIOPheonix6() {
    shooterMotor1 =
        new TalonFX(Constants.MotorCANIds.shooterMotor1CANId, Constants.MotorCANIds.CanBusName);
    shooterMotor2 =
        new TalonFX(Constants.MotorCANIds.shooterMotor2CANId, Constants.MotorCANIds.CanBusName);
    shooterMotor3 =
        new TalonFX(Constants.MotorCANIds.shooterMotor3CANId, Constants.MotorCANIds.CanBusName);
    hoodMotor = new TalonFX(Constants.MotorCANIds.hoodMotorCANId, Constants.MotorCANIds.CanBusName);

    // Slot0: 强PID（稳速用）
    Slot0Configs shooterSlot0 = new Slot0Configs();
    shooterSlot0.kP = Constants.ShooterSubsystemPID.shooterKP;
    shooterSlot0.kI = Constants.ShooterSubsystemPID.shooterKI;
    shooterSlot0.kD = Constants.ShooterSubsystemPID.shooterKD;
    shooterSlot0.kS = Constants.ShooterSubsystemPID.shooterKS;
    shooterSlot0.kV = Constants.ShooterSubsystemPID.shooterKV;

    // Slot1: 弱PID（混合控制用）
    com.ctre.phoenix6.configs.Slot1Configs shooterSlot1 =
        new com.ctre.phoenix6.configs.Slot1Configs();
    shooterSlot1.kP = 0.5;
    shooterSlot1.kI = 0.0;
    shooterSlot1.kD = 0.01;
    shooterSlot1.kS = 0.0;
    shooterSlot1.kV = 0.01;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        Constants.ShooterSubsystemPID.shooterPeakReverseTorqueCurrent;
    shooterConfig.Slot0 = shooterSlot0;
    shooterConfig.Slot1 = shooterSlot1;

    TalonFXConfiguration shooterConfigInverted = new TalonFXConfiguration();
    shooterConfigInverted.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfigInverted.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfigInverted.Slot0 = shooterSlot0;
    shooterConfigInverted.Slot1 = shooterSlot1;
    shooterMotor1.getConfigurator().apply(shooterConfig);
    shooterMotor2.getConfigurator().apply(shooterConfig);
    shooterMotor3.getConfigurator().apply(shooterConfigInverted);
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Slot0 = new Slot0Configs();
    hoodConfig.Slot0.kP = Constants.ShooterSubsystemPID.hoodKP;
    hoodConfig.Slot0.kI = Constants.ShooterSubsystemPID.hoodKI;
    hoodConfig.Slot0.kD = Constants.ShooterSubsystemPID.hoodKD;
    hoodConfig.Slot0.kS = Constants.ShooterSubsystemPID.hoodKS;
    hoodConfig.Slot0.kV = Constants.ShooterSubsystemPID.hoodKV;
    hoodConfig.MotionMagic = new MotionMagicConfigs();
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.ShooterSubsystemPID.hoodMotionMagicCruiseVelocity;
    hoodConfig.MotionMagic.MotionMagicAcceleration =
        Constants.ShooterSubsystemPID.hoodMotionMagicAcceleration;
    hoodConfig.MotionMagic.MotionMagicJerk = Constants.ShooterSubsystemPID.hoodMotionMagicJerk;
    hoodConfig.Feedback.SensorToMechanismRatio = ShooterSubsystemPID.hoodGearRatio;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void ShooterSetRps(double rps, double feedforwardAmps) {
    shooterMotor1.setControl(
        shooterVelocityRequest.withVelocity(rps).withFeedForward(feedforwardAmps));
    shooterMotor2.setControl(
        shooterVelocityRequest.withVelocity(rps).withFeedForward(feedforwardAmps));
    shooterMotor3.setControl(
        shooterVelocityRequest.withVelocity(rps).withFeedForward(feedforwardAmps));
  }

  @Override
  public void ShooterSetRpsWithSlot(double rps, int slot, double feedforwardAmps) {
    shooterMotor1.setControl(
        shooterVelocityRequest.withVelocity(rps).withSlot(slot).withFeedForward(feedforwardAmps));
    shooterMotor2.setControl(
        shooterVelocityRequest.withVelocity(rps).withSlot(slot).withFeedForward(feedforwardAmps));
    shooterMotor3.setControl(
        shooterVelocityRequest.withVelocity(rps).withSlot(slot).withFeedForward(feedforwardAmps));
  }

  @Override
  public void ShooterSetCurrent(double amps) {
    shooterMotor1.setControl(shooterCurrentRequest.withOutput(amps));
    shooterMotor2.setControl(shooterCurrentRequest.withOutput(amps));
    shooterMotor3.setControl(shooterCurrentRequest.withOutput(amps));
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV) {
    Slot0Configs shooterSlot0 = new Slot0Configs();
    shooterSlot0.kP = kP;
    shooterSlot0.kI = kI;
    shooterSlot0.kD = kD;
    shooterSlot0.kS = kS;
    shooterSlot0.kV = kV;
    shooterMotor1.getConfigurator().apply(shooterSlot0);
    shooterMotor2.getConfigurator().apply(shooterSlot0);
    shooterMotor3.getConfigurator().apply(shooterSlot0);
  }

  @Override
  public void setPIDSlot1(double kP, double kI, double kD, double kS, double kV) {
    com.ctre.phoenix6.configs.Slot1Configs shooterSlot1 =
        new com.ctre.phoenix6.configs.Slot1Configs();
    shooterSlot1.kP = kP;
    shooterSlot1.kI = kI;
    shooterSlot1.kD = kD;
    shooterSlot1.kS = kS;
    shooterSlot1.kV = kV;
    shooterMotor1.getConfigurator().apply(shooterSlot1);
    shooterMotor2.getConfigurator().apply(shooterSlot1);
    shooterMotor3.getConfigurator().apply(shooterSlot1);
  }

  @Override
  public void ShooterSetV(double voltage) {
    shooterMotor1.setControl(shooterVoltageRequest.withOutput(voltage));
    shooterMotor2.setControl(shooterVoltageRequest.withOutput(voltage));
    shooterMotor3.setControl(shooterVoltageRequest.withOutput(voltage));
  }

  @Override
  public void HoodSetAngle(double angle) {
    double torque = hoodMotor.getTorqueCurrent().getValueAsDouble();
    double velocity = Math.abs(hoodMotor.getVelocity().getValueAsDouble());

    boolean isOverLoaded = (torque > 100) && (velocity < 0.015);
    if (!isOverLoaded) { // Use Motion Magic only when the motor is not over-loaded
      hoodMotor.setControl(hoodMotionMagicRequest.withPosition(angle));
    }
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
    inputs.ShooterTorqueCurrent =
        (shooterMotor1.getTorqueCurrent().getValueAsDouble()
                + shooterMotor2.getTorqueCurrent().getValueAsDouble()
                + shooterMotor3.getTorqueCurrent().getValueAsDouble())
            / 3.0;
    inputs.HoodAngle = hoodMotor.getPosition().getValueAsDouble();
    inputs.HoodVoltageV = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.HoodCurrentAMPS = hoodMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPeakReverseTorque(double current) {
    TorqueCurrentConfigs config = new TorqueCurrentConfigs();
    // Read current config to avoid overwriting other values
    shooterMotor1.getConfigurator().refresh(config);
    config.PeakReverseTorqueCurrent = current;
    shooterMotor1.getConfigurator().apply(config);

    shooterMotor2.getConfigurator().refresh(config);
    config.PeakReverseTorqueCurrent = current;
    shooterMotor2.getConfigurator().apply(config);

    shooterMotor3.getConfigurator().refresh(config);
    config.PeakReverseTorqueCurrent = current;
    shooterMotor3.getConfigurator().apply(config);
  }
}
