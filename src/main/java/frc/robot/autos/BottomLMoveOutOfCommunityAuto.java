package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.ReverseIntakeAuto;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class BottomLMoveOutOfCommunityAuto extends SequentialCommandGroup {

    private final int direction;

    public BottomLMoveOutOfCommunityAuto(Swerve swerve, GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem, boolean isRed) {
        direction = isRed ? -1 : 1;
        addCommands(
                // Zero arm
                new CalibrateArmCommand(grabberSubsystem),
                // Move arm to front pole
                new MoveToPresetArmPosition(grabberSubsystem, ArmPositions.FRONT_POLE),
                // Score cone
                new ParallelCommandGroup(
                    new ReverseIntakeAuto(intakeSubsystem, swerve),
                    new MoveRobotAuto(swerve, new Translation2d(0.05, 0.0), 0.5)),
                // Move out of community and zero arm.
                new ParallelCommandGroup(
                    new MoveRobotAuto(swerve, new Translation2d(0.2, 0.0), 3.0),
                    new CalibrateArmCommand(grabberSubsystem)),
                // Move to behind charging station
                new MoveRobotAuto(swerve, new Translation2d(0, direction * 0.12), 2.0),
                // Move onto charging station
                new MoveRobotAuto(swerve, new Translation2d(-0.2, 0.0), 1.5),
                // Level the charging station
                new LevelChargingStationAuto(swerve));

    }

}
