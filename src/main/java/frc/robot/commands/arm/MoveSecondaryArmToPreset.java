package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;

public class MoveSecondaryArmToPreset extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;

    public MoveSecondaryArmToPreset(ArmSubsystem armSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(targetPosition.getPrimaryArmAngle(), targetPosition.getSecondaryArmAngle());
        armSubsystem.moveSecondaryToTarget();
    }

    @Override
    public void execute() {
        armSubsystem.moveSecondaryToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedSecondaryTarget();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSecondaryMotor(0);
    }

}
