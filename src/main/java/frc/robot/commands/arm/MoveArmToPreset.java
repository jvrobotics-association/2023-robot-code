package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPreset extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;

    public MoveArmToPreset(ArmSubsystem armSubsystem, ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValue(targetPosition.getArmAngle());
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

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
    }

}
