package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClawCommand extends CommandBase {

    // Required subsystems
    private final ClawSubsystem clawSubsystem;

    public CloseClawCommand(ClawSubsystem _ClawSubsystem){
        this.clawSubsystem = _ClawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.closeClaw();
    }
    
}
