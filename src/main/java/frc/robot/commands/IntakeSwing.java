package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakeSwing extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double dgr1, dgr2;

  private enum STAT {
    At_dgr1,
    moving,
    At_dgr2
  };

  private STAT state = STAT.At_dgr1;

  public IntakeSwing(IntakeSubsystem intakeSubsystem, double dgr1, double dgr2) {
    this.intakeSubsystem = intakeSubsystem;
    this.dgr1 = dgr1;
    this.dgr2 = dgr2;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double current_dgr = intakeSubsystem.getPivotAngleDegrees();
    if (dgr1 + 2 < current_dgr && state == STAT.At_dgr1
        || dgr2 - 2 > current_dgr && state == STAT.At_dgr2) {
      state = STAT.moving;
    } else if (Math.abs(dgr2 - current_dgr) <= 2 && state == STAT.moving) {
      state = STAT.At_dgr2;
    } else if (Math.abs(dgr1 - current_dgr) <= 2 && state == STAT.moving) {
      state = STAT.At_dgr1;
    }
    switch (state) {
      case At_dgr1:
        intakeSubsystem.setPivotAngle(dgr2);
        break;
      case At_dgr2:
        intakeSubsystem.setPivotAngle(dgr1);
    }

    // intakeSubsystem.setPivotAngle(dgr1);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setPivotAngle(dgr1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
