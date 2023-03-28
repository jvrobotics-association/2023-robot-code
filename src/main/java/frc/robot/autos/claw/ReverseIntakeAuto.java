package frc.robot.autos.claw;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeAuto extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private double startTime;

    public ReverseIntakeAuto(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeMotor(0.5);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
    
}
