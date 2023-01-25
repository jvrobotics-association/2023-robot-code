package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.EnableRotateAroundPointCommand;

public class DriverControls {
    private final Joystick driveJoystick;
    private final Joystick controlPannel;

    public DriverControls(int drivePortNumer, int controlPortNumber) {
        driveJoystick = new Joystick(drivePortNumer);
        controlPannel = new Joystick(controlPortNumber);

        new JoystickButton(driveJoystick, 3).whileTrue(new EnableRotateAroundPointCommand());
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
