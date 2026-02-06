package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

public class Aimbot extends Command {
  private Drive drive;
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private double dist;
  private PIDController aimpid = new PIDController(5, 0, 0);
  private boolean swing = false;
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
      IntakeSubsystem intakeSubsystem) {
    this.drive = drive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(drive);
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);

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
        feederSubsystem.setIndexerVoltage(12);
        feederSubsystem.setBeltVoltage(12);
        swing = true;
      } // } else {
      //   feederSubsystem.setIndexerVoltage(0);
      //   feederSubsystem.setBeltVoltage(0);
      // }
    }
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

    // SmartDashboard.putNumber("rps", targetRPS);
    // SmartDashboard.putNumber("hood_angle", targetHoodAngle);
    ChassisSpeeds aimspeed = new ChassisSpeeds(0, 0, turnOutput);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setIndexerVoltage(0);
    shooterSubsystem.setShooterVoltage(0);
    feederSubsystem.setBeltVoltage(0);
    intakeSubsystem.setPivotAngle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
