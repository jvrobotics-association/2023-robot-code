package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;

public class SecondaryDriveControls {

    private final XboxController controller;

    public SecondaryDriveControls (int port) {
        controller = new XboxController(port);
    }

    public double getForward() {
        return controller.getRawAxis(1);
    }

    public double getStrafe() {
        return controller.getRawAxis(0);
    }

    public double getYaw() {
        return controller.getRawAxis(2);
    }

}