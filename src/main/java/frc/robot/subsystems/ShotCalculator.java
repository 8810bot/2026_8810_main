package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends SubsystemBase {
    private final Drive drive;

    // Output results
    private Pose3d effectiveTarget = new Pose3d();
    private double effectiveYaw = 0.0;
    private double shooterRps = 0.0;
    private double hoodAngle = 0.0;

    public ShotCalculator(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        // 1. Get Robot State
        Pose2d robotPose = drive.getPose();
        var robotVel = drive.getFieldRelativeSpeeds();
        var robotAccel = drive.getFieldRelativeAccelerations();

        // 2. Determine Real Target (assuming Blue Goal for now, ideally switch based on Alliance)
        // Constants.aimconstants.bluegoalpos is Translation2d, we need Pose3d
        // Assuming goal height is ~2.0m (needs verification from game manual)
        double goalHeight = 2.0;
        Pose3d realTarget = new Pose3d(
            Constants.aimconstants.bluegoalpos.getX(),
            Constants.aimconstants.bluegoalpos.getY(),
            goalHeight,
            new Rotation3d()
        );

        // 3. Calculate distance to estimate ball speed
        // This is a simplification. In reality, you'd iterate or have a lookup table for ball speed too.
        double dist = robotPose.getTranslation().getDistance(Constants.aimconstants.bluegoalpos);

        // TODO: Get real ball speed from a lookup table in Constants
        double estimatedBallSpeed = 20.0; // m/s placeholder

        // 4. Run SOTF Algorithm
        // Robot pose for SOTF usually needs to be the shooter's pose, not the drive base center
        // But for now we use robotPose. Ideally add a Transform3d for the shooter offset.
        Pose3d shooterPose = new Pose3d(robotPose);

        InterceptSolution solution = ShootOnTheFlyCalculator.solveShootOnTheFly(
            shooterPose,
            realTarget,
            robotVel,
            robotAccel,
            estimatedBallSpeed,
            5,    // iterations
            0.01  // tolerance
        );

        // 5. Update Results
        this.effectiveTarget = solution.effectiveTargetPose();

        // Calculate required chassis yaw to face the virtual target
        double dx = effectiveTarget.getX() - robotPose.getX();
        double dy = effectiveTarget.getY() - robotPose.getY();
        this.effectiveYaw = Math.atan2(dy, dx);

        // Calculate shooting parameters based on the VIRTUAL distance
        double virtualDist = new Translation2d(dx, dy).getNorm();

        // Lookup RPS and Angle based on virtual distance
        if (Constants.distanceToRps.size() > 0) {
            this.shooterRps = Constants.distanceToRps.get(virtualDist);
        } else {
            this.shooterRps = 3000; // Default fallback
        }

        if (Constants.distanceToAngle.size() > 0) {
            this.hoodAngle = Constants.distanceToAngle.get(virtualDist);
        } else {
            this.hoodAngle = 45; // Default fallback
        }

        // Logging
        Logger.recordOutput("ShotCalculator/EffectiveTarget", effectiveTarget);
        Logger.recordOutput("ShotCalculator/EffectiveYaw", effectiveYaw);
        Logger.recordOutput("ShotCalculator/VirtualDistance", virtualDist);
        Logger.recordOutput("ShotCalculator/ShooterRPS", shooterRps);
        Logger.recordOutput("ShotCalculator/HoodAngle", hoodAngle);
    }

    public double getEffectiveYaw() { return effectiveYaw; }
    public double getShooterRps() { return shooterRps; }
    public double getHoodAngle() { return hoodAngle; }
}
