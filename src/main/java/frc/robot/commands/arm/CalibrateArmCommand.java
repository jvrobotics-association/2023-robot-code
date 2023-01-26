package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem = RobotContainer.getArmSubsystem();

    /**
     * This command sets the arm to the zero position.
     */
    public CalibrateArmCommand() {
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setPrimaryMotor(0.3);
        armSubsystem.setSecondaryMotor(0.3);
    }

    // Returns whether the arm is stopped.
    // This is used to check for when the arm is done moving.
    // The arm is done moving when both motors are stopped.
    @Override
    public boolean isFinished() {
        return armSubsystem.isPrimaryMotorStopped() && armSubsystem.isSecondaryMotorStopped();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPrimaryMotor(0);
        armSubsystem.setSecondaryMotor(0);
        armSubsystem.resetEncoders();
    }

}
