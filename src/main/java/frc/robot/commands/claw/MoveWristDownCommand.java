package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveWristDownCommand extends CommandBase {

    private final GrabberSubsystem grabberSubsystem;

    public MoveWristDownCommand(GrabberSubsystem _grabberSubsystem) {
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setWristMotor(0.15);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return grabberSubsystem.isWristMotorStoppedDown();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.stopWristMotor();
    }
}
