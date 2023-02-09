package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class GoToFortyFiveCommand extends CommandBase {

    // Required subsystems
    private final ArmSubsystem armSubsystem = RobotContainer.getArmSubsystem();

    public GoToFortyFiveCommand() {
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setTargetEncoderValues(new Translation2d(1.22131483, 1.22131483));
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.hasReachedTarget();
    }
    
}
