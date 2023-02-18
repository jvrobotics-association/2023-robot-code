package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("Set Claw X", 0.762);
        SmartDashboard.putNumber("Set Claw Y", 0.9144);

        armSubsystem.setTargetEncoderValues(new Translation2d(
            SmartDashboard.getNumber("Set Claw X", 0.762),
            SmartDashboard.getNumber("Set Claw Y", 0.9144)
        ));
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
