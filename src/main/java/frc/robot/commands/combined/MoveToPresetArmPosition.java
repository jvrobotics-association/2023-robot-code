package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.GrabberSubsystem;


/*
 * TODO: Make the movement sequential to prioritize specific motions first
 * For example this would be move secondary up then primary forward for scoring
 * This prevents colliding with frame and game elements.
 */

public class MoveToPresetArmPosition extends SequentialCommandGroup {

    private final ArmPositions targetPosition;
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
