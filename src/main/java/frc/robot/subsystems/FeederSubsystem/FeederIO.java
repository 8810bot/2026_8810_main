package frc.robot.subsystems.FeederSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  public default void BeltSetRps(double RPS) {}

  public default void IndexerSetRps(double RPS) {}

  public default void BeltSetV(double voltage) {}

  public default void IndexerSetV(double voltage) {}

  public default void setStatorCurrentLimit(double amps) {}

  @AutoLog
  public class FeederIOInputs {
    public double BeltVelocityRPS = 0;
    public double IndexerVelocityRPS = 0;

    public double BeltCurrentAMPS = 0;
    public double IndexerCurrentAMPS = 0;
    public double BeltVoltageV = 0;
    public double IndexerVoltageV = 0;
    public double IndexerMechanismRPS = 0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
}
