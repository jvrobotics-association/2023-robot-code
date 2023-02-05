package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.claw.CloseClawCommand;
import frc.robot.commands.claw.OpenClawCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.claw.RunClawIntakeCommand;
import frc.robot.commands.drive.EnableRotateAroundPointCommand;

public class DriverControls {
    // The joystick for the driver and the control panel
    private final Joystick driveJoystick;
    private final Joystick controlPannel;

    public DriverControls(int drivePortNumer, int controlPortNumber) {
        // Initialize the joysticks
        driveJoystick = new Joystick(drivePortNumer);
        controlPannel = new Joystick(controlPortNumber);

        // Assign the buttons to commands
        new JoystickButton(driveJoystick, 3).whileTrue(new EnableRotateAroundPointCommand());
        new JoystickButton(driveJoystick, 7).whileTrue(new RunClawIntakeCommand());
        new JoystickButton(driveJoystick, 9).whileTrue(new ReverseClawIntakeCommand());
        new JoystickButton(driveJoystick, 8).whileTrue(new OpenClawCommand());
        new JoystickButton(driveJoystick, 10).whileTrue(new CloseClawCommand());
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
