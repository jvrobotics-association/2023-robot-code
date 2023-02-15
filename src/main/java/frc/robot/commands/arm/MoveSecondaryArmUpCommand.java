package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveSecondaryArmUpCommand extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem;

    public MoveSecondaryArmUpCommand(ArmSubsystem _armSubsystem) {
        this.armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setSecondaryMotor(0.3);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSecondaryMotor(0);
    }
    
}
