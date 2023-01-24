package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends CommandBase {
    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();
    private final DriverControls driverControls = RobotContainer.getDriverControls();

    public TeleopDriveCommand() {
        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        drivetrain.drive(driverControls.getForward(), driverControls.getStrafe(), driverControls.getYaw(), false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}