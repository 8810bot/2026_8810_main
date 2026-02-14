package frc.robot.subsystems.ShooterSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public default void ShooterSetRps(double rps, double feedforwardAmps) {}

  public default void ShooterSetRpsWithSlot(double rps, int slot, double feedforwardAmps) {}

  public default void ShooterSetCurrent(double amps) {}

  public default void ShooterSetV(double voltage) {}

  public default void HoodSetAngle(double angle) {}

  public default void HoodSetZero() {}

  public default void HoodSetV(double voltage) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV) {}

  public default void setPIDSlot1(double kP, double kI, double kD, double kS, double kV) {}

  public default void setPeakReverseTorque(double current) {}

  public default void setFollowerEnabled(boolean enabled) {}

  public default void setStatorCurrentLimit(double amps) {}

  public default void setHoodStatorCurrentLimit(double amps) {}

  @AutoLog
  public class ShooterIOInputs {
    public double ShooterRPS = 0;
    public double ShooterCurrentAMPS = 0;
    public double ShooterTorqueCurrent = 0;
    public double ShooterClosedLoopOutput = 0;
    public double ShooterClosedLoopFeedForward = 0;
    public double ShooterClosedLoopError = 0;
    public double ShooterClosedLoopReference = 0;
    public double HoodAngle = 0;
    public double HoodVoltageV = 0;
    public double HoodCurrentAMPS = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
