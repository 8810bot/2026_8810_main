// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final class aimconstants {
    public static final Translation2d bluegoalpos = new Translation2d(4.6256, 4.0345);
    public static final Translation2d redgoalpos = new Translation2d(11.915, 4.0345);
    public static final double fieldmid = 4.0345;
    public static final double downtrench = 0.66599;
    public static final double uptrench = 7.40334;
    public static final double autonShootTimeoutSeconds = 3.0;

    public static InterpolatingDoubleTreeMap distanceToShooterRPS =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
      distanceToShooterRPS.put(0.834, 50.0);
      distanceToShooterRPS.put(1.318, 50.0);
      distanceToShooterRPS.put(1.877, 53.0);
      distanceToShooterRPS.put(2.384, 55.0);
      distanceToShooterRPS.put(2.873, 57.0);
      distanceToShooterRPS.put(3.305, 58.0);
      distanceToShooterRPS.put(3.853, 59.0);
      distanceToShooterRPS.put(4.365, 61.0);
      distanceToShooterRPS.put(4.866, 63.0);

      distanceToHoodAngle.put(0.834, 0.0);
      distanceToHoodAngle.put(1.318, 4.0);
      distanceToHoodAngle.put(1.877, 7.0);
      distanceToHoodAngle.put(2.384, 10.0);
      distanceToHoodAngle.put(2.873, 11.0);
      distanceToHoodAngle.put(3.305, 13.0);
      distanceToHoodAngle.put(3.853, 17.0);
      distanceToHoodAngle.put(4.365, 21.0);
      distanceToHoodAngle.put(4.866, 23.0);
    }
  }

  public static final class MotorCANIds {
    public static final String CanBusName = "mainCAN";
    public static final int shooterMotor1CANId = 50;
    public static final int shooterMotor2CANId = 51;
    public static final int shooterMotor3CANId = 52;
    public static final int hoodMotorCANId = 53;
    public static final int beltMotorCANId = 41;
    public static final int indexerMotorCANId = 40;
    public static final int intakePivotMotorCANId = 61;
    public static final int intakeMotorCANId = 60;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class PoseEstimatorConstants {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.3, 0.3, 0.1);
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final InterpolatingDoubleTreeMap tAtoDev = new InterpolatingDoubleTreeMap();

    static {
      tAtoDev.put(0.17, 0.08);
      tAtoDev.put(0.12, 0.20);
      tAtoDev.put(0.071, 0.35);
      tAtoDev.put(0.046, 0.4);
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FeederSubsystemPID {
    public static final double indexerKP = 1;
    public static final double indexerKI = 0;
    public static final double indexerKD = 0;
    public static final double indexerKS = 0;
    public static final double indexerKV = 0;
    public static final double beltKP = 1;
    public static final double beltKI = 0;
    public static final double beltKD = 0;
    public static final double beltKS = 0;
    public static final double beltKV = 0;
  }

  public static class IntakePID {
    public static final double intakeKP = 2;
    public static final double intakeKI = 0;
    public static final double intakeKD = 0;
    public static final double intakeKS = 0;
    public static final double intakeKV = 0;
    public static final double pivotKP = 200;
    public static final double pivotKI = 1;
    public static final double pivotKD = 0;
    public static final double pivotKS = 0.4;
    public static final double pivotKV = 0;
    public static final double pivotKG = 0.2;
    public static final double pivotGearRatio = 53.3;
    public static final double pivotMotionMagicCruiseVelocity = 0.4;
    public static final double pivotMotionMagicAcceleration = 6.4;
    public static final double pivotMotionMagicJerk = 0;
  }

  public static class ShooterSubsystemPID {
    public static final double shooterKP = 6;
    public static final double shooterKI = 0;
    public static final double shooterKD = 0.1;
    public static final double shooterKS = 0;
    public static final double shooterKV = 0;
    public static final double hoodGearRatio = 8. * 13.;
    public static final double hoodKP = 1000;
    public static final double hoodKI = 0;
    public static final double hoodKD = 200;
    public static final double hoodKS = 0;
    public static final double hoodKV = 0;

    public static final double hoodMotionMagicCruiseVelocity = 0.5;
    public static final double hoodMotionMagicAcceleration = 4;
    public static final double hoodMotionMagicJerk = 0;
  }
}
