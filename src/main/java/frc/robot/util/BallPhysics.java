package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class BallPhysics {
  public static final double GRAVITY = 9.81;

  public record ShotSolution(double launchPitchRad, double launchSpeed, double flightTimeSeconds) {}

  private BallPhysics() {}

  public static ShotSolution solveBallisticWithSpeed(
      Pose3d shooterPose, Pose3d targetPose, double launchSpeed) {

    Translation3d s = shooterPose.getTranslation();
    Translation3d t = targetPose.getTranslation();

    double dx = t.getX() - s.getX();
    double dy = t.getY() - s.getY();
    double dz = t.getZ() - s.getZ();

    double d = Math.hypot(dx, dy);
    if (d < 1e-9) {
      throw new IllegalArgumentException("Horizontal distance too small");
    }

    double v2 = launchSpeed * launchSpeed;
    double g = GRAVITY;

    double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
    if (discriminant < 0) {
      return new ShotSolution(0, 0, 0);
    }

    // LOW-ARC solution (use +Math.sqrt(...) for high arc)
    double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

    double launchPitch = Math.atan(tanTheta);

    double vHoriz = launchSpeed * Math.cos(launchPitch);
    double time = d / vHoriz;

    return new ShotSolution(launchPitch, launchSpeed, time);
  }
}
