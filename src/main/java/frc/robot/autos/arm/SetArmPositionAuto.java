package frc.robot.autos.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionAuto extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;

    public SetArmPositionAuto(ArmSubsystem armSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(targetPosition.getPrimaryArmAngle(), targetPosition.getSecondaryArmAngle());
        armSubsystem.moveToTarget();
    }

    @Override
    public void execute() {
        armSubsystem.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedTarget();
    }
}
