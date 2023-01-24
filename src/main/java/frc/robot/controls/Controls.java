package frc.robot.controls;

public class Controls {
    private final DriverControls driverControls;

    public Controls() {
        driverControls = new DriverControls(0, 1);
    }

    public DriverControls getDriverControls() {
        return driverControls;
    }
}
