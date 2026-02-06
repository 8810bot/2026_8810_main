// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimandDrive;
import frc.robot.commands.Aimbot;
import frc.robot.commands.AutonTrench;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeSwing;
import frc.robot.commands.PivotInit;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // path planner

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public final LoggedTunableNumber ShooterTestRPS = new LoggedTunableNumber("SHooterRPS", 60);
  public final LoggedTunableNumber HoodAngle = new LoggedTunableNumber("HoodAngle", 0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public FeederSubsystem feederSubsystem = new FeederSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, a nd commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "AIMandShoot", new Aimbot(drive, shooterSubsystem, feederSubsystem, intakeSubsystem));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Wheel Rotation Tests
    autoChooser.addOption(
        "Rotate All Modules to 0 Degrees",
        DriveCommands.rotateModulesToAngle(drive, Rotation2d.fromDegrees(0)));
    autoChooser.addOption(
        "Rotate All Modules to 90 Degrees",
        DriveCommands.rotateModulesToAngle(drive, Rotation2d.fromDegrees(90)));
    autoChooser.addOption(
        "Rotate All Modules to 45 Degrees",
        DriveCommands.rotateModulesToAngle(drive, Rotation2d.fromDegrees(45)));
    autoChooser.addOption(
        "Rotate FL Module to 90 Degrees",
        DriveCommands.rotateModuleToAngle(drive, 0, Rotation2d.fromDegrees(90)));
    autoChooser.addDefaultOption("default", new PathPlannerAuto("New Auto"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getRawAxis(1),
            () -> -controller.getRawAxis(0),
            () -> -controller.getRawAxis(4)));

    // Lock to 0° when A button is held
    controller
        .rightBumper()
        .onTrue(new InstantCommand(() -> intakeSubsystem.setPivotZero()).ignoringDisable(true));
    controller
        .leftBumper()
        .whileTrue(new InstantCommand(() -> intakeSubsystem.setIntakeRps(60)))
        .onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeVoltage(0)));
    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .x()
        .onTrue(
            new InstantCommand(() -> shooterSubsystem.setHoodZero())
                .ignoringDisable(true)
                .alongWith(new InstantCommand(() -> intakeSubsystem.setPivotZero())));
    controller
        .povUp()
        .whileTrue(
            new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0)), drive)
                .alongWith(new WaitCommand(20)));
    controller
        .povRight()
        .whileTrue(
            new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(0, 1, 0)), drive)
                .alongWith(new WaitCommand(20)));
    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    controller
        .leftTrigger()
        .whileTrue(
            new AimandDrive(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller
        .rightStick()
        .whileTrue(new AutonTrench(drive, shooterSubsystem, () -> controller.getLeftY()));

    controller
        .rightTrigger()
        .whileTrue(new Aimbot(drive, shooterSubsystem, feederSubsystem, intakeSubsystem));

    // test intake swing
    controller.y().whileTrue(new IntakeSwing(intakeSubsystem, 0, 40));
    controller.a().onTrue(new PivotInit(intakeSubsystem));
    // D-Pad controls for fine translation (0.5x max speed, Field-Relative)
    // Forward (Up)
    // controller
    //     .povUp()
    //     .whileTrue(
    //         Commands.run(
    //             () ->
    //                 drive.runVelocity(
    //                     ChassisSpeeds.fromFieldRelativeSpeeds(
    //                         drive.getMaxLinearSpeedMetersPerSec() * 0.5,
    //                         0.0,
    //                         0.0,
    //                         drive.getRotation())),
    //             drive));
    // // Backward (Down)
    // controller
    //     .povDown()
    //     .whileTrue(
    //         Commands.run(
    //             () ->
    //                 drive.runVelocity(
    //                     ChassisSpeeds.fromFieldRelativeSpeeds(
    //                         -drive.getMaxLinearSpeedMetersPerSec() * 0.5,
    //                         0.0,
    //                         0.0,
    //                         drive.getRotation())),
    //             drive));
    // // Left (Left)
    // controller
    //     .povLeft()
    //     .whileTrue(
    //         Commands.run(
    //             () ->
    //                 drive.runVelocity(
    //                     ChassisSpeeds.fromFieldRelativeSpeeds(
    //                         0.0,
    //                         drive.getMaxLinearSpeedMetersPerSec() * 0.5,
    //                         0.0,
    //                         drive.getRotation())),
    //             drive));
    // // Right (Right)
    // controller
    //     .povRight()
    //     .whileTrue(
    //         Commands.run(
    //             () ->
    //                 drive.runVelocity(
    //                     ChassisSpeeds.fromFieldRelativeSpeeds(
    //                         0.0,
    //                         -drive.getMaxLinearSpeedMetersPerSec() * 0.5,
    //                         0.0,
    //                         drive.getRotation())),
    //             drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
