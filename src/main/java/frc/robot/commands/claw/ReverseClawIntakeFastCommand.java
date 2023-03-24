package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class ReverseClawIntakeFastCommand extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem grabberSubsystem;

    public ReverseClawIntakeFastCommand(GrabberSubsystem _grabberSubsystem){
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setIntakeMotor(1.00);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.stopIntakeMotor();
    }
    
}
