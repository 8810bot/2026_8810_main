package frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim flywheelSim = new FlywheelSim(
      DCMotor.getKrakenX60(2),
      1.0,
      0.01 // J kg*m^2
  );
  private final PIDController flywheelController = new PIDController(0.1, 0.0, 0.0);
  private boolean flywheelClosedLoop = false;
  private double flywheelAppliedVolts = 0.0;

  private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
      DCMotor.getFalcon500(1),
      100.0,
      SingleJointedArmSim.estimateMOI(0.3, 2.0), // Length 0.3m, Mass 2kg
      0.3, // Length
      Units.degreesToRadians(0.0), // Min angle
      Units.degreesToRadians(60.0), // Max angle
      true, // Simulate gravity
      Units.degreesToRadians(0.0) // Start angle
  );
  private final PIDController hoodController = new PIDController(5.0, 0.0, 0.0);
  private boolean hoodClosedLoop = false;
  private double hoodAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update Flywheel
    if (flywheelClosedLoop) {
      double pidVolts = flywheelController.calculate(flywheelSim.getAngularVelocityRPM() / 60.0);
      flywheelAppliedVolts = Math.max(-12.0, Math.min(12.0, pidVolts));
    }
    
    flywheelSim.setInputVoltage(flywheelAppliedVolts);
    flywheelSim.update(0.02);
    
    inputs.ShooterRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
    inputs.ShooterCurrentAMPS = flywheelSim.getCurrentDrawAmps();
    
    // Update Hood
    if (hoodClosedLoop) {
      double pidVolts = hoodController.calculate(hoodSim.getAngleRads());
      hoodAppliedVolts = Math.max(-12.0, Math.min(12.0, pidVolts));
    }
    
    hoodSim.setInputVoltage(hoodAppliedVolts);
    hoodSim.update(0.02);
    
    inputs.HoodAngle = Units.radiansToDegrees(hoodSim.getAngleRads());
    inputs.HoodVoltageV = hoodAppliedVolts;
    inputs.HoodCurrentAMPS = hoodSim.getCurrentDrawAmps();
  }

  @Override
  public void ShooterSetRps(double rps) {
    flywheelClosedLoop = true;
    flywheelController.setSetpoint(rps);
  }

  @Override
  public void ShooterSetV(double voltage) {
    flywheelClosedLoop = false;
    flywheelAppliedVolts = Math.max(-12.0, Math.min(12.0, voltage));
  }

  @Override
  public void HoodSetAngle(double angle) {
    hoodClosedLoop = true;
    hoodController.setSetpoint(Units.degreesToRadians(angle));
  }

  @Override
  public void HoodSetZero() {
    hoodSim.setState(0.0, 0.0);
  }

  @Override
  public void HoodSetV(double voltage) {
    hoodClosedLoop = false;
    hoodAppliedVolts = Math.max(-12.0, Math.min(12.0, voltage));
  }
}
