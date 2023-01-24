package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;

public class DriverControls {
    private final Joystick driveJoystick;
    private final Joystick controlPannel;

    public DriverControls(int drivePortNumer, int controlPortNumber) {
        driveJoystick = new Joystick(drivePortNumer);
        controlPannel = new Joystick(controlPortNumber);
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
