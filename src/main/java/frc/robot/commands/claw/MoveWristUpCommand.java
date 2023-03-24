package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveWristUpCommand extends CommandBase {

    // Required subsystems
    private final GrabberSubsystem grabberSubsystem;

    public MoveWristUpCommand(GrabberSubsystem _grabberSubsystem) {
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (grabberSubsystem.isWristMotorStoppedUp())
            return;
        grabberSubsystem.setWristMotor(-0.15);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return grabberSubsystem.isWristMotorStoppedUp();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.stopWristMotor();
    }
    
}