package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class InverseKinematicsCommand extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem;

    public InverseKinematicsCommand(ArmSubsystem _armSubsystem) {
        this.armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(new Translation2d(0.8, 0.8));
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
