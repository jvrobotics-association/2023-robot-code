package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ClawSubsystem;

public class MoveWristToPreset extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final ArmPositions targetPosition;

    public MoveWristToPreset(ClawSubsystem clawSubsystem, ArmPositions targetPosition) {
        this.clawSubsystem = clawSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setWristEncoderTarget(targetPosition.getWristAngle());
        clawSubsystem.moveToTarget();
    }

    @Override
    public void execute() {
        clawSubsystem.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return clawSubsystem.hasReachedTarget();
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setWristMotor(0);
    }
    
}
