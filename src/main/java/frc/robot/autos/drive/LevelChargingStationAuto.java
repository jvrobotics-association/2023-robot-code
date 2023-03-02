package frc.robot.autos.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LevelChargingStationAuto extends CommandBase {

    private final Swerve swerve;

    public LevelChargingStationAuto(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.levelRobotPitch();        
    }

    @Override
    public void execute() {
        swerve.levelRobotPitch();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, true);
    }
    
}
