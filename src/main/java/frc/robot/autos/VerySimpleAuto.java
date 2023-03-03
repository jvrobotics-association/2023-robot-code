package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.ReverseIntakeAuto;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Swerve;

public class VerySimpleAuto extends SequentialCommandGroup{
    

    public VerySimpleAuto(Swerve swerve, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, boolean isRed) {
        addCommands(
            new MoveToPresetArmPosition(armSubsystem, clawSubsystem, ArmPositions.FRONT_POLE),
            new MoveRobotAuto(swerve, isRed ? 1 : -1, 1.0),
            new ReverseIntakeAuto(clawSubsystem),
            new CalibrateArmCommand(armSubsystem, clawSubsystem),
            new MoveRobotAuto(swerve, isRed ? -1 : 1, 2.0),
            new LevelChargingStationAuto(swerve)
        );
    }
}
