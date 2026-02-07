package frc.robot.subsystems.ShooterSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public default void ShooterSetRps(double rps, double feedforwardAmps) {}

  public default void ShooterSetV(double voltage) {}

  public default void HoodSetAngle(double angle) {}

  public default void HoodSetZero() {}

  public default void HoodSetV(double voltage) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV) {}

  public default void setPeakReverseTorque(double current) {}

  @AutoLog
  public class ShooterIOInputs {
    public double ShooterRPS = 0;
    public double ShooterCurrentAMPS = 0;
    public double ShooterTorqueCurrent = 0;
    public double HoodAngle = 0;
    public double HoodVoltageV = 0;
    public double HoodCurrentAMPS = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
