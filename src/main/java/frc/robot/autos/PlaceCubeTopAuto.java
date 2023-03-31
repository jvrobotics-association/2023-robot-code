package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.claw.IntakeAuto;
import frc.robot.autos.claw.ReverseIntakeFastAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class PlaceCubeTopAuto extends SequentialCommandGroup {

    public PlaceCubeTopAuto(IntakeSubsystem intakeSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(new IntakeAuto(intakeSubsystem, 0.25),
        // Zero arm
        new CalibrateArmCommand(grabberSubsystem),
        // Move arm to front pole
        new MoveToPresetArmPosition(grabberSubsystem, ArmPositions.BACK_SHELF),
        // Score cone
        new ReverseIntakeFastAuto(intakeSubsystem, 1.0),
        // Rezero arm
        new CalibrateArmCommand(grabberSubsystem));
    }
}
