package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class CalibrateArmCommand extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem grabberSubsystem;

    /**
     * This command sets the arm to the zero position.
     */
    public CalibrateArmCommand(GrabberSubsystem _grabberSubsystem) {
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        // grabberSubsystem.setPrimaryMotor(-Constants.Arm.primaryArmMaxSpeed);
        // grabberSubsystem.setSecondaryMotor(-Constants.Arm.secondaryArmMaxSpeed);
        grabberSubsystem.setArmMotor(Constants.Arm.maxSpeed);
        grabberSubsystem.setWristMotor(Constants.Claw.wristMotorSpeed);
        // grabberSubsystem.resetEncoders();
    }

    // Returns whether the arm is stopped.
    // This is used to check for when the arm is done moving.
    // The arm is done moving when both motors are stopped.
    @Override
    public boolean isFinished() {
        // return grabberSubsystem.isPrimaryMotorStoppedBackwards() & grabberSubsystem.isSecondaryMotorStoppedDown() & grabberSubsystem.isWristMotorStoppedUp();
        // return grabberSubsystem.isPrimaryMotorStoppedForward();
        return grabberSubsystem.isArmMotorStoppedForward() & grabberSubsystem.isWristMotorStoppedDown();
    }

    @Override
    public void end(boolean interrupted) {
        // grabberSubsystem.setPrimaryMotor(0);
        // grabberSubsystem.setSecondaryMotor(0);
        grabberSubsystem.setArmMotor(0);
        grabberSubsystem.setWristMotor(0);
        grabberSubsystem.resetEncoder();
        grabberSubsystem.resetEncoder();
    }

}
