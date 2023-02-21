package frc.robot.commands.april_tag;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;

public class UpdateRobotPositionCommand extends CommandBase {

    private final AprilTagSubsystem aprilTagSubsystem;

    public UpdateRobotPositionCommand(AprilTagSubsystem aprilTagSubsystem) {
        this.aprilTagSubsystem = aprilTagSubsystem;
        addRequirements(aprilTagSubsystem);
    }

    @Override
    public void execute() {
        aprilTagSubsystem.updateRobotPosition();
    }
    
}
