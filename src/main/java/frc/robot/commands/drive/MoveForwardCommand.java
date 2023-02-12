package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class MoveForwardCommand extends CommandBase {

    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();

    public MoveForwardCommand () {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize () {
        drivetrain.spinDriveMotors(0.3);
        drivetrain.rotateMotorsForward();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.spinDriveMotors(0);
    }
    
}
