package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;

public class MovePrimaryArmToPreset extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;

    public MovePrimaryArmToPreset(ArmSubsystem armSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(targetPosition.getPrimaryArmAngle(), targetPosition.getSecondaryArmAngle());
        armSubsystem.movePrimaryToTarget();
    }

    @Override
    public void execute() {
        armSubsystem.movePrimaryToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedPrimaryTarget();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPrimaryMotor(0);
    }

}
