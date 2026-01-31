package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class AimandDrive extends Command {
  private Drive drive;
  private DoubleSupplier xSupplier, ySupplier;
  PIDController aimpid = new PIDController(5, 0, 0);

  public AimandDrive(Drive m_drive, DoubleSupplier i_xSupplier, DoubleSupplier i_ySupplier) {
    drive = m_drive;
    addRequirements(drive);
    xSupplier = i_xSupplier;
    ySupplier = i_ySupplier;
    aimpid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());
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

    // Rotate robot
    ChassisSpeeds aimspeed =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            turnOutput);
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimspeed, pose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
