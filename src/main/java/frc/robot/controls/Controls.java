package frc.robot.controls;

import frc.robot.Constants;

public class Controls {
    // The controls for the driver and the secondary driver
    private final DriverControls driverControls;
    private final SecondaryDriveControls secondaryDriveControls;

    Controls() {
        // Initialize the controls
        driverControls = new DriverControls(Constants.Controllers.DRIVER_CONTROLS_PORT, Constants.Controllers.CONTROL_PANEL_PORT);
        secondaryDriveControls = new SecondaryDriveControls(Constants.Controllers.SECONDARY_DRIVER_CONTROLS_PORT);
    }

    public DriverControls getDriverControls() {
        return driverControls;
    }

    public SecondaryDriveControls getSecondaryDriveControls() {
        return secondaryDriveControls;
    }
}
