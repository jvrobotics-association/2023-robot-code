package frc.robot.controls;

import frc.robot.Constants;

public class Controls {
    private final DriverControls driverControls;
    private final SecondaryDriveControls secondaryDriveControls;

    Controls() {
        driverControls = new DriverControls(Constants.Controllers.DRIVER_CONTROLS_PORT, Constants.Controllers.CONTROL_PANEL_PORT);
        secondaryDriveControls = new SecondaryDriveControls(Constants.Controllers.SECONDARY_DRIVER_CONTROLS_PORT);
    }

    public DriverControls getDriverControls() {
        return driverControls;
    }
}
