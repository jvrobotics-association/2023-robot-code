package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class CalibrateArmCommand extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;

    /**
     * This command sets the arm to the zero position.
     */
    public CalibrateArmCommand(ArmSubsystem _armSubsystem, ClawSubsystem _clawSubsystem) {
        this.armSubsystem = _armSubsystem;
        this.clawSubsystem = _clawSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setPrimaryMotor(-Constants.Arm.primaryArmMaxSpeed);
        armSubsystem.setSecondaryMotor(-Constants.Arm.secondaryArmMaxSpeed);
        clawSubsystem.setWristMotor(-Constants.Claw.wristMotorSpeed);
        // armSubsystem.resetEncoders();
    }

    // Returns whether the arm is stopped.
    // This is used to check for when the arm is done moving.
    // The arm is done moving when both motors are stopped.
    @Override
    public boolean isFinished() {
        return armSubsystem.isPrimaryMotorStoppedBackwards() & armSubsystem.isSecondaryMotorStoppedDown() & clawSubsystem.isWristMotorStoppedUp();
        // return armSubsystem.isPrimaryMotorStoppedForward();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPrimaryMotor(0);
        armSubsystem.setSecondaryMotor(0);
        clawSubsystem.setWristMotor(0);
        armSubsystem.resetEncoders();
        clawSubsystem.resetEncoder();
    }

}
