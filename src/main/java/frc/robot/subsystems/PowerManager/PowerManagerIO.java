package frc.robot.subsystems.PowerManager;

import org.littletonrobotics.junction.AutoLog;

public interface PowerManagerIO {
  @AutoLog
  class PowerManagerIOInputs {
    public double batteryVoltage = 12.0;
    public double totalCurrent = 0.0;
    public double[] channelCurrents = new double[24];
    public double temperature = 0.0;
  }

  default void updateInputs(PowerManagerIOInputs inputs) {}
}
