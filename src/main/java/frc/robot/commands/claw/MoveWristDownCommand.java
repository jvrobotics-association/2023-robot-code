package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class MoveWristDownCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public MoveWristDownCommand(ClawSubsystem _clawSubsystem) {
        this.clawSubsystem = _clawSubsystem;
        addRequirements(clawSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (clawSubsystem.isWristMotorStoppedDown())
            return;
        clawSubsystem.setWristMotor(0.3);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return clawSubsystem.isWristMotorStoppedDown();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopWristMotor();
    }
}
