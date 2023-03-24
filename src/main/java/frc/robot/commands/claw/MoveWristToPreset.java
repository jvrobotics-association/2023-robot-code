package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveWristToPreset extends CommandBase {

    private final GrabberSubsystem grabberSubsystem;
    private final ArmPositions targetPosition;

    public MoveWristToPreset(GrabberSubsystem grabberSubsystem, ArmPositions targetPosition) {
        this.grabberSubsystem = grabberSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setWristEncoderTarget(targetPosition.getWristAngle());
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
        grabberSubsystem.setWristMotor(0);
    }
    
}
