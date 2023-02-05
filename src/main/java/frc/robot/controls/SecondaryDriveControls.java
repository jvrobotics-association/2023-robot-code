package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.arm.GoToFortyFiveCommand;
import frc.robot.commands.arm.MovePrimaryArmBackwardsCommand;
import frc.robot.commands.arm.MovePrimaryArmForwardCommand;
import frc.robot.commands.arm.MoveSecondaryArmDownCommand;
import frc.robot.commands.arm.MoveSecondaryArmUpCommand;

public class SecondaryDriveControls {
    // The controller for the secondary driver primarily used for the arm
    private final XboxController controller;

    public SecondaryDriveControls(int port) {
        // Initialize the controller
        controller = new XboxController(port);

        // Assign buttons to commands
        new JoystickButton(controller, 5).whileTrue(new MovePrimaryArmForwardCommand());
        new JoystickButton(controller, 6).whileTrue(new MovePrimaryArmBackwardsCommand());
        new JoystickButton(controller, 3).whileTrue(new MoveSecondaryArmUpCommand());
        new JoystickButton(controller, 2).whileTrue(new MoveSecondaryArmDownCommand());
        new JoystickButton(controller, 4).whileTrue(new CalibrateArmCommand());
        new JoystickButton(controller, 7).whileTrue(new GoToFortyFiveCommand());
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