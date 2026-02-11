package frc.robot.subsystems.PowerManager;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PowerManager extends SubsystemBase {
  private final PowerManagerIO io;
  private final PowerManagerIOInputsAutoLogged inputs = new PowerManagerIOInputsAutoLogged();

  // Tunable thresholds
  private final LoggedTunableNumber warningVoltage =
      new LoggedTunableNumber("PowerManager/Config/WarningVoltage", 10.5);
  private final LoggedTunableNumber dangerVoltage =
      new LoggedTunableNumber("PowerManager/Config/DangerVoltage", 8.0);

  // Profile Limits (Tunable)
  // IDLE
  private final LoggedTunableNumber idleDrive =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Drive", 40.0);
  private final LoggedTunableNumber idleSteer =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Steer", 20.0);
  private final LoggedTunableNumber idleShooter =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Shooter", 10.0);
  private final LoggedTunableNumber idleFeeder =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Feeder", 10.0);
  private final LoggedTunableNumber idleIntake =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Intake", 10.0);
  private final LoggedTunableNumber idleHood =
      new LoggedTunableNumber("PowerManager/Profiles/IDLE/Hood", 10.0);

  // DRIVING
  private final LoggedTunableNumber drivingDrive =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Drive", 120.0);
  private final LoggedTunableNumber drivingSteer =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Steer", 60.0);
  private final LoggedTunableNumber drivingShooter =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Shooter", 10.0);
  private final LoggedTunableNumber drivingFeeder =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Feeder", 10.0);
  private final LoggedTunableNumber drivingIntake =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Intake", 10.0);
  private final LoggedTunableNumber drivingHood =
      new LoggedTunableNumber("PowerManager/Profiles/DRIVING/Hood", 10.0);

  // SHOOTING
  private final LoggedTunableNumber shootingDrive =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Drive", 40.0);
  private final LoggedTunableNumber shootingSteer =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Steer", 40.0);
  private final LoggedTunableNumber shootingShooter =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Shooter", 100.0);
  private final LoggedTunableNumber shootingFeeder =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Feeder", 40.0);
  private final LoggedTunableNumber shootingIntake =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Intake", 20.0);
  private final LoggedTunableNumber shootingHood =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOTING/Hood", 20.0);

  // SHOOT_AND_DRIVE
  private final LoggedTunableNumber shootDriveDrive =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Drive", 80.0);
  private final LoggedTunableNumber shootDriveSteer =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Steer", 50.0);
  private final LoggedTunableNumber shootDriveShooter =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Shooter", 80.0);
  private final LoggedTunableNumber shootDriveFeeder =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Feeder", 30.0);
  private final LoggedTunableNumber shootDriveIntake =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Intake", 15.0);
  private final LoggedTunableNumber shootDriveHood =
      new LoggedTunableNumber("PowerManager/Profiles/SHOOT_AND_DRIVE/Hood", 15.0);

  // AUTO
  private final LoggedTunableNumber autoDrive =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Drive", 100.0);
  private final LoggedTunableNumber autoSteer =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Steer", 60.0);
  private final LoggedTunableNumber autoShooter =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Shooter", 90.0);
  private final LoggedTunableNumber autoFeeder =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Feeder", 35.0);
  private final LoggedTunableNumber autoIntake =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Intake", 15.0);
  private final LoggedTunableNumber autoHood =
      new LoggedTunableNumber("PowerManager/Profiles/AUTO/Hood", 15.0);

  // Current state
  private final LoggedDashboardChooser<PowerProfile> profileChooser =
      new LoggedDashboardChooser<>("PowerManager/ProfileChooser");
  private PowerProfile overrideProfile = null; // null means use chooser
  private PowerProfile currentProfile = PowerProfile.IDLE;
  private final PowerDistributionState currentState = new PowerDistributionState();

  // Min currents (absolute floor for safety)
  private static final double MIN_DRIVE_CURRENT = 20.0;
  private static final double MIN_STEER_CURRENT = 20.0;
  private static final double MIN_SHOOTER_CURRENT = 10.0;
  private static final double MIN_FEEDER_CURRENT = 10.0;
  private static final double MIN_INTAKE_CURRENT = 10.0;
  private static final double MIN_HOOD_CURRENT = 5.0;

  public static class PowerDistributionState {
    public double driveCurrentLimit = 40.0;
    public double steerCurrentLimit = 30.0;
    public double shooterCurrentLimit = 10.0;
    public double feederCurrentLimit = 10.0;
    public double intakeCurrentLimit = 10.0;
    public double hoodLimit = 10.0;
  }

  public PowerManager(PowerManagerIO io) {
    this.io = io;

    // Set up profile chooser
    for (PowerProfile profile : PowerProfile.values()) {
      profileChooser.addOption(profile.toString(), profile);
    }
    profileChooser.addDefaultOption("Default (IDLE)", PowerProfile.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PowerManager", inputs);

    // Determine target profile (Override takes precedence over Dashboard)
    if (overrideProfile != null) {
      currentProfile = overrideProfile;
    } else {
      currentProfile = profileChooser.get();
    }

    // Calculate throttle factor based on voltage
    double voltage = inputs.batteryVoltage;
    double warnV = warningVoltage.get();
    double dangerV = dangerVoltage.get();
    double throttleFactor = 1.0;

    // Simple hysteresis could be added here if needed, but linear throttling is usually stable
    // enough
    if (voltage < dangerV) {
      throttleFactor = 0.0;
    } else if (voltage < warnV) {
      throttleFactor = (voltage - dangerV) / (warnV - dangerV);
    }

    // Update state based on profile and throttle
    updateDistributionState(throttleFactor);

    // Log status
    Logger.recordOutput("PowerManager/Profile", currentProfile.toString());
    Logger.recordOutput("PowerManager/ThrottleFactor", throttleFactor);
    Logger.recordOutput("PowerManager/Limits/Drive", currentState.driveCurrentLimit);
    Logger.recordOutput("PowerManager/Limits/Steer", currentState.steerCurrentLimit);
    Logger.recordOutput("PowerManager/Limits/Shooter", currentState.shooterCurrentLimit);
    Logger.recordOutput("PowerManager/Limits/Feeder", currentState.feederCurrentLimit);
    Logger.recordOutput("PowerManager/Limits/Intake", currentState.intakeCurrentLimit);
    Logger.recordOutput("PowerManager/Limits/Hood", currentState.hoodLimit);
  }

  private void updateDistributionState(double throttleFactor) {
    double driveTarget = 0.0;
    double steerTarget = 0.0;
    double shooterTarget = 0.0;
    double feederTarget = 0.0;
    double intakeTarget = 0.0;
    double hoodTarget = 0.0;

    switch (currentProfile) {
      case IDLE:
        driveTarget = idleDrive.get();
        steerTarget = idleSteer.get();
        shooterTarget = idleShooter.get();
        feederTarget = idleFeeder.get();
        intakeTarget = idleIntake.get();
        hoodTarget = idleHood.get();
        break;
      case DRIVING:
        driveTarget = drivingDrive.get();
        steerTarget = drivingSteer.get();
        shooterTarget = drivingShooter.get();
        feederTarget = drivingFeeder.get();
        intakeTarget = drivingIntake.get();
        hoodTarget = drivingHood.get();
        break;
      case SHOOTING:
        driveTarget = shootingDrive.get();
        steerTarget = shootingSteer.get();
        shooterTarget = shootingShooter.get();
        feederTarget = shootingFeeder.get();
        intakeTarget = shootingIntake.get();
        hoodTarget = shootingHood.get();
        break;
      case SHOOT_AND_DRIVE:
        driveTarget = shootDriveDrive.get();
        steerTarget = shootDriveSteer.get();
        shooterTarget = shootDriveShooter.get();
        feederTarget = shootDriveFeeder.get();
        intakeTarget = shootDriveIntake.get();
        hoodTarget = shootDriveHood.get();
        break;
      case AUTO:
        driveTarget = autoDrive.get();
        steerTarget = autoSteer.get();
        shooterTarget = autoShooter.get();
        feederTarget = autoFeeder.get();
        intakeTarget = autoIntake.get();
        hoodTarget = autoHood.get();
        break;
    }

    currentState.driveCurrentLimit = calculateLimit(driveTarget, MIN_DRIVE_CURRENT, throttleFactor);
    currentState.steerCurrentLimit = calculateLimit(steerTarget, MIN_STEER_CURRENT, throttleFactor);
    currentState.shooterCurrentLimit =
        calculateLimit(shooterTarget, MIN_SHOOTER_CURRENT, throttleFactor);
    currentState.feederCurrentLimit =
        calculateLimit(feederTarget, MIN_FEEDER_CURRENT, throttleFactor);
    currentState.intakeCurrentLimit =
        calculateLimit(intakeTarget, MIN_INTAKE_CURRENT, throttleFactor);
    currentState.hoodLimit = calculateLimit(hoodTarget, MIN_HOOD_CURRENT, throttleFactor);
  }

  private double calculateLimit(double profileLimit, double minLimit, double factor) {
    return minLimit + (profileLimit - minLimit) * factor;
  }

  /**
   * Sets a temporary override profile.
   *
   * @param profile The profile to enforce, or null to return to Dashboard control.
   */
  public void setProfile(PowerProfile profile) {
    this.overrideProfile = profile;
  }

  public PowerDistributionState getCurrentState() {
    return currentState;
  }
}
