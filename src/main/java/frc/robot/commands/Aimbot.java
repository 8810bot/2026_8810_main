package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

public class Aimbot extends Command {
  private Drive drive;
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private double dist;
  private PIDController aimpid = new PIDController(5, 0, 0);

  public Aimbot(Drive drive, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    this.drive = drive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;

    addRequirements(drive);
    addRequirements(shooterSubsystem);
    addRequirements(feederSubsystem);

    aimpid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    aimpid.setTolerance(0.087);
    dist = Drive.distanceToGoal;
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double robotX = pose.getX();
    double robotY = pose.getY();
    double robotYaw = pose.getRotation().getRadians(); // radians
    double dx, dy = 0;
    // Vector from robot to goal
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      dx = Constants.aimconstants.bluegoalpos.getX() - robotX;
      dy = Constants.aimconstants.bluegoalpos.getY() - robotY;
    } else {
      dx = Constants.aimconstants.redgoalpos.getX() - robotX;
      dy = Constants.aimconstants.redgoalpos.getY() - robotY;
    }
    // Target angle to goal (field frame)
    double targetAngle = Math.atan2(dy, dx); // radians

    // Tell PID controller that inputs are circular

    // Compute output
    double turnOutput = aimpid.calculate(robotYaw, targetAngle);
    if (aimpid.atSetpoint()) {
      turnOutput = 0;
      double targetRPS = (double) Constants.aimconstants.distanceToShooterRPS.get(dist);
      double targetHoodAngle = (double) Constants.aimconstants.distanceToHoodAngle.get(dist);
      drive.stopWithX();
      shooterSubsystem.setShooterRps(targetRPS);
      shooterSubsystem.setHoodAngle(targetHoodAngle);
      if (shooterSubsystem.isAtSetSpeed(targetRPS)) {
        feederSubsystem.setIndexerVoltage(8);
      } else {
        feederSubsystem.setIndexerVoltage(0);
      }
    }

    // SmartDashboard.putNumber("rps", targetRPS);
    // SmartDashboard.putNumber("hood_angle", targetHoodAngle);
    ChassisSpeeds aimspeed = new ChassisSpeeds(0, 0, turnOutput);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIndexerVoltage(0);
    shooterSubsystem.setShooterVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
