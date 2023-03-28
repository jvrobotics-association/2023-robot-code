package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunClawIntakeCommand extends CommandBase {

    // Required subsystems
    private final IntakeSubsystem intakeSubsystem;

    public RunClawIntakeCommand(IntakeSubsystem _intakeSubsystem){
        this.intakeSubsystem = _intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeMotor(-1);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
    
}
