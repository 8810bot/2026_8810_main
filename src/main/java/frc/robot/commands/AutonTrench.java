package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.util8810;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutonTrench extends Command {
  private Drive drive;
  private DoubleSupplier xSupplier;
  PIDController aimPidController = new PIDController(5.0, 0.0, 0.0);
  PIDController yPidController = new PIDController(5.0, 0, 0);
  private double y_target;
  private double rot_target;
  private ShooterSubsystem shooterSubsystem;

  public AutonTrench(
      Drive m_drive, ShooterSubsystem m_shooterSubsystem, DoubleSupplier i_xSupplier) {
    drive = m_drive;
    shooterSubsystem = m_shooterSubsystem;
    addRequirements(drive);
    xSupplier = i_xSupplier;
    yPidController.setTolerance(0.1);
    aimPidController.enableContinuousInput(-Math.PI, Math.PI);
    aimPidController.setTolerance(Math.toRadians(5));
  }

  @Override
  public void initialize() {
    Pose2d pose = drive.getPose();
    double robotY = pose.getY();
    double robotYaw = pose.getRotation().getRadians();
    if (robotY < Constants.aimconstants.fieldmid) {
      y_target = Constants.aimconstants.downtrench;
    } else if (robotY > Constants.aimconstants.fieldmid) {
      y_target = Constants.aimconstants.uptrench;
    } else {
      y_target = Constants.aimconstants.fieldmid;
    }
    if (robotYaw < (Math.PI / 2) && robotYaw > -(Math.PI / 2)) {
      rot_target = 0;
    } else {
      rot_target = Math.PI;
    }
    shooterSubsystem.setHoodAngle(0);
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double robotY = pose.getY();
    double robotYaw = pose.getRotation().getRadians();
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), 0);
    double rot_speed = aimPidController.calculate(robotYaw, rot_target);
    double y_speed = yPidController.calculate(robotY, y_target);
    y_speed = yPidController.atSetpoint() ? 0 : y_speed;
    rot_speed = aimPidController.atSetpoint() ? 0 : rot_speed;
    ChassisSpeeds aimspeed =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            util8810.clamp(y_speed, -2, 2),
            rot_speed);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
    Logger.recordOutput("AutoTrench/IsYAtSetpoint", yPidController.atSetpoint());
    Logger.recordOutput("AutoTrench/IsYawAtSetpoint", aimPidController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
