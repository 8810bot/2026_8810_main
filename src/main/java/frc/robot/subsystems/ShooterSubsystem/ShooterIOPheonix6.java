package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ShooterSubsystemPID;

public class ShooterIOPheonix6 implements ShooterIO {
  private static TalonFX shooterMotor1, shooterMotor2, shooterMotor3, hoodMotor;
  private final VelocityTorqueCurrentFOC shooterVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
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

    Slot0Configs shooterSlot0 = new Slot0Configs();
    shooterSlot0.kP = Constants.ShooterSubsystemPID.shooterKP;
    shooterSlot0.kI = Constants.ShooterSubsystemPID.shooterKI;
    shooterSlot0.kD = Constants.ShooterSubsystemPID.shooterKD;
    shooterSlot0.kS = Constants.ShooterSubsystemPID.shooterKS;
    shooterSlot0.kV = Constants.ShooterSubsystemPID.shooterKV;

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0 = shooterSlot0;

    shooterMotor1.getConfigurator().apply(shooterConfig);
    shooterMotor2.getConfigurator().apply(shooterConfig);
    shooterMotor3.getConfigurator().apply(shooterConfig);
    shooterMotor2.setControl(
        new Follower(Constants.MotorCANIds.shooterMotor1CANId, MotorAlignmentValue.Aligned));
    shooterMotor3.setControl(
        new Follower(Constants.MotorCANIds.shooterMotor1CANId, MotorAlignmentValue.Opposed));
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
  public void ShooterSetRps(double rps) {
    shooterMotor1.setControl(shooterVelocityRequest.withVelocity(rps));
  }

  @Override
  public void ShooterSetV(double voltage) {
    shooterMotor1.setControl(shooterVoltageRequest.withOutput(voltage));
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
    inputs.HoodAngle = hoodMotor.getPosition().getValueAsDouble();
    inputs.HoodVoltageV = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.HoodCurrentAMPS = hoodMotor.getSupplyCurrent().getValueAsDouble();
  }
}
