package java.frc.robot.controls;

import edu.wpi.first.wpilibj.Controller;

public class SecondaryDriveControls {

    private final Controller controller;

    SecondaryDriveControls () {
        controller = new Controller(Constants.Controllers.SECONDARY_DRIVER_CONTROLS_PORT);
    }

    public double getForward() {
        return driveJoystick.getRawAxis(1);
    }

    public double getStrafe() {
        return driveJoystick.getRawAxis(0);
    }

    public double getYaw() {
        return driveJoystick.getRawAxis(2);
    }

}