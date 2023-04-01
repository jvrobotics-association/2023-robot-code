package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.IntakeAuto;
import frc.robot.autos.claw.ReverseIntakeAuto;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class CenterLevelAuto extends SequentialCommandGroup {


    public CenterLevelAuto(Swerve swerve, GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeAuto(intakeSubsystem, 0.25),
                // Zero arm
                new CalibrateArmCommand(grabberSubsystem),
                // Move arm to front pole
                new MoveToPresetArmPosition(grabberSubsystem, ArmPositions.FRONT_POLE),
                // Score cone
                new ReverseIntakeAuto(intakeSubsystem, swerve, 1.0),
                // Move out of community over the and zero arm.
                new ParallelCommandGroup(
                    new MoveRobotAuto(swerve, new Translation2d(0.15, 0.0), 1.5),
                    new CalibrateArmCommand(grabberSubsystem)),
                
                // Level charging station once on the 
                new LevelChargingStationAuto(swerve));

    }
    
}