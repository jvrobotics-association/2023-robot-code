package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;

public class ReverseClawIntakeCommand extends CommandBase {

    // Required subsystems
    private final ClawSubsystem clawSubsystem = RobotContainer.getClawSubsystem();

    public ReverseClawIntakeCommand(){}

    @Override
    public void initialize() {
        clawSubsystem.setIntakeMotor(1);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopIntakeMotor();
    }
    
}
