package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotXAuto;
import frc.robot.autos.drive.MoveRobotYAuto;
import frc.robot.subsystems.Swerve;

public class NoArmAuto extends SequentialCommandGroup {

    public NoArmAuto(Swerve swerve, boolean isRed) {
        addCommands(
            new MoveRobotXAuto(swerve, isRed ? -1 : 1, 1.0),
            new MoveRobotYAuto(swerve, isRed ? -1 : 1, 1.0),
            new LevelChargingStationAuto(swerve));
      
    }
    
}
