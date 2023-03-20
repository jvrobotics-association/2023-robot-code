package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

/*
 * TODO: Make the movement sequential to prioritize specific motions first
 * For example this would be move secondary up then primary forward for scoring
 * This prevents colliding with frame and game elements.
 */

public class MoveToPresetArmPosition extends SequentialCommandGroup {

    private final ArmSubsystem armSubsystem;
    private final ArmPositions targetPosition;
    private final ClawSubsystem clawSubsystem;

    public MoveToPresetArmPosition(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem,
            ArmPositions targetPosition) {
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem, clawSubsystem);

        addCommands(
                // new MoveAllToTargetCommand(armSubsystem, clawSubsystem, ArmPositions.KNOWN_GOOD_CONFIGURATION),

                new MoveAllToTargetCommand(armSubsystem, clawSubsystem, targetPosition));
    }

}
