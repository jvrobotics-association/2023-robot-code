package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class MoveToPresetArmPosition extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;
    private final ClawSubsystem clawSubsystem;

    public MoveToPresetArmPosition(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(targetPosition.getPrimaryArmAngle(), targetPosition.getSecondaryArmAngle());
        armSubsystem.moveToTarget();
        clawSubsystem.setWristEncoderTarget(targetPosition.getWristAngle());
        clawSubsystem.moveToTarget();
    }

    @Override
    public void execute() {
        armSubsystem.moveToTarget();
        clawSubsystem.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedTarget() & clawSubsystem.hasReachedTarget();
    }
    
    
}
