package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class RunClawIntakeCommand extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem grabberSubsystem;

    public RunClawIntakeCommand(GrabberSubsystem _grabberSubsystem){
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setIntakeMotor(-1);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.stopIntakeMotor();
    }
    
}
