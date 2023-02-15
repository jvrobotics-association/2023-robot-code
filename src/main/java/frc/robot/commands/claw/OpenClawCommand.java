package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class OpenClawCommand extends CommandBase {

    // Required subsystems
    private final ClawSubsystem clawSubsystem;

    public OpenClawCommand(ClawSubsystem _clawSubsystem){
        this.clawSubsystem = _clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.openClaw();
    }

}
