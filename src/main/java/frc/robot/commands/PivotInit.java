package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class PivotInit extends Command {
    private IntakeSubsystem intakeSubsystem;
    double torque_current;
    double velocity;
    private double pivotHomeStartSec = 0.0;
    private final static double PIVOT_HOME_TIMEOUT_SECONDS = 5.0;
    private boolean isOverLoaded = false;
    private boolean isHomed = false;
    public PivotInit(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        pivotHomeStartSec = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
        if(!isOverLoaded){
            intakeSubsystem.setPivotVoltage(-2.0);
            torque_current = intakeSubsystem.getPivotCurrentAmps();
            velocity = Math.abs(intakeSubsystem.getIntakeVelocityRps());
            isOverLoaded =
            (torque_current > Constants.IntakePID.pivotThresholdCurrentAmps)
                && (velocity < 0.015) || (Timer.getFPGATimestamp() - pivotHomeStartSec > PIVOT_HOME_TIMEOUT_SECONDS);
        }
        else{
            isHomed = true;
        }
    }

    public boolean isHomed() {
        return isHomed;
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setPivotVoltage(0.0);
        intakeSubsystem.setPivotZero();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}