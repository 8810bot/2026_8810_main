package frc.robot.subsystems.PowerManager;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerManagerIOReal implements PowerManagerIO {
  private final PowerDistribution pdh;

  public PowerManagerIOReal() {
    // Rev PDH default module is 1
    pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void updateInputs(PowerManagerIOInputs inputs) {
    inputs.batteryVoltage = pdh.getVoltage();
    inputs.totalCurrent = pdh.getTotalCurrent();
    for (int i = 0; i < 24; i++) {
      inputs.channelCurrents[i] = pdh.getCurrent(i);
    }
    inputs.temperature = pdh.getTemperature();
  }
}
