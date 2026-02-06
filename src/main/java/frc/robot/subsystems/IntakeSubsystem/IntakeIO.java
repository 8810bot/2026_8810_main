package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  public default void intakesetrps(double RPS) {}

  public default void IntakesetV(double voltage) {}

  public default void Pivotsetangle(double degrees) {}

  public default void pivotsetV(double voltage) {}

  public default void pivotSetZero() {}

  @AutoLog
  public class IntakeIOInputs {
    public double intakeVelocityRPS = 0;
    public double PivotAngledegrees = 0;

    public double intakeCurrentAMPS = 0;
    public double pivotCurrentAMPS = 0;
    public double PivotVelocityRPS = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
