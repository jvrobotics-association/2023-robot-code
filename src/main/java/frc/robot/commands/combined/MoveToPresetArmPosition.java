package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveToPresetArmPosition extends SequentialCommandGroup {

    @SuppressWarnings("unused")
    private final ArmPositions targetPosition;
    @SuppressWarnings("unused")
    private final GrabberSubsystem grabberSubsystem;

    public MoveToPresetArmPosition(GrabberSubsystem grabberSubsystem, ArmPositions targetPosition) {
        this.grabberSubsystem = grabberSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(grabberSubsystem, grabberSubsystem);

        addCommands(
                // new MoveAllToTargetCommand(grabberSubsystem, grabberSubsystem, ArmPositions.KNOWN_GOOD_CONFIGURATION),

                new MoveAllToTargetCommand(grabberSubsystem, targetPosition));
    }

}
