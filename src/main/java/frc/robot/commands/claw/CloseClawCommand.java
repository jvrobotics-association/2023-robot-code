package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClawCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem = RobotContainer.getClawSubsystem();

    public CloseClawCommand(){}

    @Override
    public void initialize() {
        clawSubsystem.closeClaw();
    }
    
}
