package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveArmToPreset extends CommandBase {

    private final GrabberSubsystem grabberSubsystem;
    private final ArmPositions targetPosition;

    public MoveArmToPreset(GrabberSubsystem grabberSubsystem, ArmPositions targetPosition) {
        this.grabberSubsystem = grabberSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setArmTargetEncoderValue(targetPosition.getArmAngle());
        grabberSubsystem.moveToTarget();
    }

    @Override
    public void execute() {
        grabberSubsystem.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return grabberSubsystem.hasReachedTarget();
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setArmMotor(0);
    }

}