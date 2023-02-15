package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MovePrimaryArmForwardCommand extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem;

    public MovePrimaryArmForwardCommand(ArmSubsystem _armSubsystem) {
        this.armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setPrimaryMotor(0.3);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return armSubsystem.isPrimaryMotorStopped();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPrimaryMotor(0);
    }
    
}
