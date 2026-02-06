package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOPheonix6 implements IntakeIO {
  private static TalonFX intakeMotor, pivotMotor;
  private final VelocityTorqueCurrentFOC intakeVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut intakeVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);

  public IntakeIOPheonix6() {
    intakeMotor =
        new TalonFX(Constants.MotorCANIds.intakeMotorCANId, Constants.MotorCANIds.CanBusName);
    pivotMotor =
        new TalonFX(Constants.MotorCANIds.intakePivotMotorCANId, Constants.MotorCANIds.CanBusName);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.Slot0 = new Slot0Configs();
    intakeConfig.Slot0.kP = Constants.IntakePID.intakeKP;
    intakeConfig.Slot0.kI = Constants.IntakePID.intakeKI;
    intakeConfig.Slot0.kD = Constants.IntakePID.intakeKD;
    intakeConfig.Slot0.kS = Constants.IntakePID.intakeKS;
    intakeConfig.Slot0.kV = Constants.IntakePID.intakeKV;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotor.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = new Slot0Configs();
    pivotConfig.Slot0.kP = Constants.IntakePID.pivotKP;
    pivotConfig.Slot0.kI = Constants.IntakePID.pivotKI;
    pivotConfig.Slot0.kD = Constants.IntakePID.pivotKD;
    pivotConfig.Slot0.kS = Constants.IntakePID.pivotKS;
    pivotConfig.Slot0.kV = Constants.IntakePID.pivotKV;
    pivotConfig.Slot0.kG = Constants.IntakePID.pivotKG;
    pivotConfig.MotionMagic = new MotionMagicConfigs();
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.IntakePID.pivotMotionMagicCruiseVelocity;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        Constants.IntakePID.pivotMotionMagicAcceleration;
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.IntakePID.pivotGearRatio;
    pivotConfig.MotionMagic.MotionMagicJerk = Constants.IntakePID.pivotMotionMagicJerk;
    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.setPosition(0);
  }

  @Override
  public void intakesetrps(double RPS) {
    intakeMotor.setControl(intakeVelocityRequest.withVelocity(RPS * 14 / 30));
  }

  @Override
  public void IntakesetV(double voltage) {
    intakeMotor.setControl(intakeVoltageRequest.withOutput(voltage));
  }

  @Override
  public void Pivotsetangle(double degrees) {
    pivotMotor.setControl(pivotMotionMagicRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void pivotsetV(double voltage) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void pivotSetZero() {
    pivotMotor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocityRPS = intakeMotor.getVelocity().getValueAsDouble();
    inputs.PivotAngledegrees =
        Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    inputs.intakeCurrentAMPS = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pivotCurrentAMPS = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.PivotVelocityRPS = pivotMotor.getVelocity().getValueAsDouble();
  }
}
