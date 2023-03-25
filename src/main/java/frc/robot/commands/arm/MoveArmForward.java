package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveArmForward extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem grabberSubsystem;

    public MoveArmForward(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setArmMotor(Constants.Arm.manualMaxSpeed);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return grabberSubsystem.getArmForwardLimitSwitch();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setArmMotor(0);
        grabberSubsystem.setWristMotor(0);
    }
}