package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmForward extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem;

    public MoveArmForward(ArmSubsystem _armSubsystem) {
        this.armSubsystem = _armSubsystem;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (armSubsystem.isMotorStoppedForward())
            return;
        armSubsystem.setMotor(Constants.Arm.manualMaxSpeed);
    }

    // If the arm is stopped, the command is finished.
    @Override
    public boolean isFinished() {
        return armSubsystem.isMotorStoppedForward();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
    }
    
}
