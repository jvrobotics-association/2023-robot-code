package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.IntakeAuto;
import frc.robot.autos.claw.ReverseIntakeAuto;
import frc.robot.autos.drive.MoveRobotAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class NoLevelBottomLMoveOutOfCommunityAuto extends SequentialCommandGroup {


    public NoLevelBottomLMoveOutOfCommunityAuto(Swerve swerve, GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeAuto(intakeSubsystem, 0.5),
                // Zero arm
                new CalibrateArmCommand(grabberSubsystem),
                // Move arm to front pole
                new MoveToPresetArmPosition(grabberSubsystem, ArmPositions.FRONT_POLE),
                // Score cone
                new ReverseIntakeAuto(intakeSubsystem, swerve, 1.0),
                // Move out of community and zero arm.
                new ParallelCommandGroup(
                    new MoveRobotAuto(swerve, new Translation2d(0.2, 0.0), 2.5),
                    new CalibrateArmCommand(grabberSubsystem)));
    }

}
