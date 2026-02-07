package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Aimbot extends Command {
  private Drive drive;
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private double dist;
  private PIDController aimpid = new PIDController(5, 0, 0);
  private boolean swing = false;
  private boolean isShooting = false;
  private DoubleSupplier indexerVolts;
  private DoubleSupplier beltVolts;
  private final Timer shotTimer = new Timer();
  private final Timer speedStableTimer = new Timer();

  private enum STAT {
    At_dgr1,
    moving,
    At_dgr2
  };

  private STAT state = STAT.At_dgr1;

  private IntakeSubsystem intakeSubsystem;

  public Aimbot(
      Drive drive,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsystem intakeSubsystem,
      DoubleSupplier indexerVolts,
      DoubleSupplier beltVolts) {
    this.drive = drive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.indexerVolts = indexerVolts;
    this.beltVolts = beltVolts;
    addRequirements(drive);
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);

    aimpid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    aimpid.setTolerance(0.087);
    dist = Drive.distanceToGoal;
    isShooting = false;
    shotTimer.stop();
    shotTimer.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double robotX = pose.getX();
    double robotY = pose.getY();
    double dx, dy = 0;

    // Vector from robot to goal
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      dx = Constants.aimconstants.bluegoalpos.getX() - robotX;
      dy = Constants.aimconstants.bluegoalpos.getY() - robotY;
    } else {
      dx = Constants.aimconstants.redgoalpos.getX() - robotX;
      dy = Constants.aimconstants.redgoalpos.getY() - robotY;
    }
    // Target angle to goal (field frame)
    double targetAngle = Math.atan2(dy, dx); // radians

    double turnOutput = aimpid.calculate(drive.getRotation().getRadians(), targetAngle);
    // Dynamic distance update
    dist = Drive.distanceToGoal;

    // Calculate targets based on distance (Calculate FIRST)
    double targetRPS = (double) Constants.aimconstants.distanceToShooterRPS.get(dist);
    double targetHoodAngle = (double) Constants.aimconstants.distanceToHoodAngle.get(dist);

    if (aimpid.atSetpoint()) {
      turnOutput = 0;
      drive.stopWithX();

      // Speed Stability Check
      if (shooterSubsystem.isAtSetSpeed(targetRPS * 1.05)) {
        speedStableTimer.start();
      } else {
        speedStableTimer.stop();
        speedStableTimer.reset();
      }

      if (speedStableTimer.get() > 0.1) {
        isShooting = true;
      }
    } else {
      // Not at setpoint
      speedStableTimer.stop();
      speedStableTimer.reset();
    }

    if (isShooting) {
      feederSubsystem.setIndexerVoltage(indexerVolts.getAsDouble());
      feederSubsystem.setBeltVoltage(beltVolts.getAsDouble());
      swing = true;
      shotTimer.start();
    } else {
      if (!aimpid.atSetpoint()) {
        isShooting = false;
      }

      shotTimer.stop();
      shotTimer.reset();
    }

    double compensation = 0;
    double timeSinceStart = shotTimer.get() - ShooterSubsystem.shotDelay.get();

    // Only apply if delay has passed
    if (timeSinceStart >= 0) {
      // Periodic check: time % period < duration
      double period = ShooterSubsystem.shotFirePeriod.get();
      if (period > 0 && (timeSinceStart % period) < ShooterSubsystem.shotPulseDuration.get()) {
        compensation = ShooterSubsystem.shotCurrentFF.get();
      }
    }

    // Always apply shooter/hood targets
    shooterSubsystem.setShooterRps(targetRPS * 1.05, compensation);
    shooterSubsystem.setHoodAngle(targetHoodAngle);

    Logger.recordOutput("Aimbot/CompensationAmps", compensation);
    Logger.recordOutput("Aimbot/IsShooting", isShooting);
    Logger.recordOutput("Aimbot/ShotTimer", shotTimer.get());
    Logger.recordOutput("Aimbot/TimeSinceStart", timeSinceStart);
    Logger.recordOutput("Aimbot/AtSetpoint", aimpid.atSetpoint());

    if (swing) {
      double current_dgr = intakeSubsystem.getPivotAngleDegrees();
      if (Constants.ShooterSubsystemPID.swing_angle_1 + 2 < current_dgr && state == STAT.At_dgr1
          || Constants.ShooterSubsystemPID.swing_angle_2 - 2 > current_dgr
              && state == STAT.At_dgr2) {
        state = STAT.moving;
      } else if (Math.abs(Constants.ShooterSubsystemPID.swing_angle_2 - current_dgr) <= 2
          && state == STAT.moving) {
        state = STAT.At_dgr2;
      } else if (Math.abs(Constants.ShooterSubsystemPID.swing_angle_1 - current_dgr) <= 2
          && state == STAT.moving) {
        state = STAT.At_dgr1;
      }
      switch (state) {
        case At_dgr1:
          intakeSubsystem.setPivotAngle(Constants.ShooterSubsystemPID.swing_angle_2);
          break;
        case At_dgr2:
          intakeSubsystem.setPivotAngle(Constants.ShooterSubsystemPID.swing_angle_1);
          break;
      }
    }

    Logger.recordOutput("Aimbot/TargetRPS", targetRPS);
    Logger.recordOutput("Aimbot/TargetHoodAngle", targetHoodAngle);
    Logger.recordOutput("Aimbot/Distance", dist);
    Logger.recordOutput("Aimbot/AtSetpoint", aimpid.atSetpoint());
    Logger.recordOutput("Aimbot/IsShooting", isShooting);
    Logger.recordOutput("Aimbot/SpeedStableTimer", speedStableTimer.get());
    Logger.recordOutput("Aimbot/ShotTimer", shotTimer.get());
    Logger.recordOutput("Aimbot/PIDError", aimpid.getPositionError());
    Logger.recordOutput("Aimbot/TargetYaw", targetAngle);
    if (DriverStation.getAlliance().isPresent()) {
      Logger.recordOutput(
          "Aimbot/TargetPosition",
          new Pose2d(
              DriverStation.getAlliance().get() == Alliance.Blue
                  ? Constants.aimconstants.bluegoalpos
                  : Constants.aimconstants.redgoalpos,
              new edu.wpi.first.math.geometry.Rotation2d())); // Log target position
    }

    ChassisSpeeds aimspeed = new ChassisSpeeds(0, 0, turnOutput);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIndexerVoltage(0);
    shooterSubsystem.setShooterVoltage(0);
    feederSubsystem.setBeltVoltage(0);
    intakeSubsystem.setPivotAngle(0);
    isShooting = false;
    shotTimer.stop();
    shotTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
