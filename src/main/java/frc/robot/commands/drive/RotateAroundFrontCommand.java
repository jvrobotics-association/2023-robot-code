package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.SecondaryDriveControls;
import frc.robot.subsystems.Drivetrain;

public class RotateAroundFrontCommand extends CommandBase {

    private final SecondaryDriveControls secondaryDriveControls = RobotContainer.getSecondaryDriveControls();
    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();

    RotateAroundFrontCommand () {
        addRequirements(drivetrain);
    }
    
}
