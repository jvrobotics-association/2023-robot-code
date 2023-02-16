package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class RunClawIntakeCommand extends CommandBase {

    // Required subsystems
    private final ClawSubsystem clawSubsystem;

    public RunClawIntakeCommand(ClawSubsystem _clawSubsystem){
        this.clawSubsystem = _clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setIntakeMotor(-1);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopIntakeMotor();
    }
    
}
