package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.ReverseIntakeAuto;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotXAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class VerySimpleAuto extends SequentialCommandGroup{
    

    public VerySimpleAuto(Swerve swerve, GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem, boolean isRed, ArmPositions target) {
        addCommands(
            new MoveToPresetArmPosition(grabberSubsystem, target),
            new MoveRobotXAuto(swerve, isRed ? 1 : -1, 1.0),
            new ReverseIntakeAuto(intakeSubsystem),
            new CalibrateArmCommand(grabberSubsystem),
            new MoveRobotXAuto(swerve, isRed ? -1 : 1, 2.0),
            new LevelChargingStationAuto(swerve)
        );
    }
}
