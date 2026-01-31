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
  }

  // SOTF Interpolation Tables
  public static final InterpolatingDoubleTreeMap distanceToRps = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();

  static {
    // Example Data - NEEDS CALIBRATION
    distanceToRps.put(2.0, 3000.0);
    distanceToRps.put(3.0, 3500.0);
    distanceToRps.put(4.0, 4000.0);
    distanceToRps.put(5.0, 4500.0);
    distanceToRps.put(6.0, 5000.0);

    // Example Data - NEEDS CALIBRATION
    distanceToAngle.put(2.0, 45.0);
    distanceToAngle.put(3.0, 40.0);
    distanceToAngle.put(4.0, 35.0);
    distanceToAngle.put(5.0, 30.0);
    distanceToAngle.put(6.0, 25.0);
  }

  public static final class MotorCANIds {
    public static final String CanBusName = "mainCAN";
    public static final int shooterMotor1CANId = 1;
    public static final int shooterMotor2CANId = 2;
    public static final int shooterMotor3CANId = 3;
    public static final int hoodMotorCANId = 8;
    public static final int beltMotorCANId = 4;
    public static final int indexerMotorCANId = 5;
    public static final int intakePivotMotorCANId = 6;
    public static final int intakeMotorCANId = 7;
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
    public static final double indexerKP = 0;
    public static final double indexerKI = 0;
    public static final double indexerKD = 0;
    public static final double indexerKS = 0;
    public static final double indexerKV = 0;
    public static final double beltKP = 0;
    public static final double beltKI = 0;
    public static final double beltKD = 0;
    public static final double beltKS = 0;
    public static final double beltKV = 0;
  }

  public static class ShooterSubsystemPID {
    public static final double shooterKP = 0;
    public static final double shooterKI = 0;
    public static final double shooterKD = 0;
    public static final double shooterKS = 0;
    public static final double shooterKV = 0;

    public static final double hoodKP = 0;
    public static final double hoodKI = 0;
    public static final double hoodKD = 0;
    public static final double hoodKS = 0;
    public static final double hoodKV = 0;

    public static final double hoodMotionMagicCruiseVelocity = 0;
    public static final double hoodMotionMagicAcceleration = 0;
    public static final double hoodMotionMagicJerk = 0;
  }
}
