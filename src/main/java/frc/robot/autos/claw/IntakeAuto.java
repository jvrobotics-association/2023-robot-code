package frc.robot.autos.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem;
    private double startTime;
    private final double duration;

    public IntakeAuto(IntakeSubsystem _intakeSubsystem, double _duration) {
        this.intakeSubsystem = _intakeSubsystem;
        this.duration = _duration;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeMotor(-1.0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > duration;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
    }

}
