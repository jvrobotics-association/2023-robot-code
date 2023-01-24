package frc.robot;

import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.controls.SecondaryDriveControls;

public class RobotContainer {
    private static Drivetrain drivetrain = new Drivetrain();
    private static DriverControls driverControls = new DriverControls(Constants.Controllers.DRIVER_CONTROLS_PORT, Constants.Controllers.CONTROL_PANEL_PORT);
    private static SecondaryDriveControls secondaryDriveControls = new SecondaryDriveControls(Constants.Controllers.SECONDARY_DRIVER_CONTROLS_PORT);

    public static Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public static DriverControls getDriverControls() {
        return driverControls;
    }

    public static SecondaryDriveControls getSecondaryDriveControls() {
        return secondaryDriveControls;
    }
}
