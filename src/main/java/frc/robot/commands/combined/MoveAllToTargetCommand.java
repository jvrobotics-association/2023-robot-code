package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class MoveAllToTargetCommand extends CommandBase {
    
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ArmPositions targetPosition;

    public MoveAllToTargetCommand(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(targetPosition.getPrimaryArmAngle(), targetPosition.getSecondaryArmAngle());
        armSubsystem.movePrimaryToTarget();
        armSubsystem.moveSecondaryToTarget();
        clawSubsystem.setWristEncoderTarget(targetPosition.getWristAngle());
        clawSubsystem.moveToTarget();
    }

    @Override
    public void execute() {
        armSubsystem.movePrimaryToTarget();
        armSubsystem.moveSecondaryToTarget();
        clawSubsystem.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedPrimaryTarget() & armSubsystem.hasReachedSecondaryTarget() & clawSubsystem.hasReachedTarget();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPrimaryMotor(0);
        armSubsystem.setSecondaryMotor(0);
        clawSubsystem.setWristMotor(0);
    }
}
