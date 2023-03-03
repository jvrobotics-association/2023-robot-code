package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveSecondaryOffZeroArea extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public MoveSecondaryOffZeroArea(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(0, Constants.Arm.zeroAreaSecondaryEncoderValue);
    }

    @Override
    public void execute() {
        armSubsystem.moveSecondaryToTarget();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isSecondaryOffZeroArea();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSecondaryMotor(0);
    }
    
}
