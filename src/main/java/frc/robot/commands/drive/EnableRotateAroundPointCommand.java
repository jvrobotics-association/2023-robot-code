package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class EnableRotateAroundPointCommand extends CommandBase {

    public EnableRotateAroundPointCommand() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.setRotateAroundFront(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.setRotateAroundFront(false);
    }
    
}
