package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveArmBackward extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem armSubsystem;

    public MoveArmBackward(GrabberSubsystem _armSubsystem) {
        this.armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (armSubsystem.isArmMotorStoppedBackwards())
            return;
        armSubsystem.setArmMotor(-Constants.Arm.manualMaxSpeed);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return armSubsystem.isArmMotorStoppedBackwards();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmMotor(0);
    }
    
}
