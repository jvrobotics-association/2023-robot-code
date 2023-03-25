package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.autos.drive.MoveRobotAuto;
import frc.robot.subsystems.Swerve;

public class NoArmAuto extends SequentialCommandGroup {

    private final int direction;

    public NoArmAuto(Swerve swerve, boolean isRed) {
        direction = isRed ? -1 : 1;
        addCommands(
        //     new MoveRobotXAuto(swerve, , 1.0),
        //     new MoveRobotYAuto(swerve, isRed ? -1 : 1, 1.0),
            new MoveRobotAuto(swerve, new Translation2d(-direction * 0.2, 0.0), 2.0),
            new MoveRobotAuto(swerve, new Translation2d(0, -direction * 0.2), 1.0),
            new MoveRobotAuto(swerve, new Translation2d(direction * 0.2, 0.0), 1.0),
            new LevelChargingStationAuto(swerve));
      
    }
    
}
