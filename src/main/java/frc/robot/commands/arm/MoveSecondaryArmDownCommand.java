package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class MoveSecondaryArmDownCommand extends CommandBase {

    private final ArmSubsystem armSubsystem = RobotContainer.getArmSubsystem();

    public MoveSecondaryArmDownCommand() {
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setSecondaryMotor(-0.3);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isSecondaryMotorStopped();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSecondaryMotor(0);
    }
    
}
