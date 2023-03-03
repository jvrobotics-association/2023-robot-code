package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.arm.MovePrimaryArmToPreset;
import frc.robot.commands.arm.MoveSecondaryArmToPreset;
import frc.robot.commands.claw.MoveWristToPreset;
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

        switch (targetPosition) {
            case FLOOR_DROP:
            case FLOOR_PICKUP_TOP:
            case BACK_POLE:
                addCommands(
                        new MoveSecondaryArmToPreset(armSubsystem, targetPosition),
                        new ParallelCommandGroup(
                                new MovePrimaryArmToPreset(armSubsystem, targetPosition),
                                new MoveWristToPreset(clawSubsystem, targetPosition)));
                break;

            default:
                addCommands(
                        new MovePrimaryArmToPreset(armSubsystem, targetPosition),
                        new ParallelCommandGroup(
                                new MoveSecondaryArmToPreset(armSubsystem, targetPosition),
                                new MoveWristToPreset(clawSubsystem, targetPosition)));
                break;
        }
    }

}
