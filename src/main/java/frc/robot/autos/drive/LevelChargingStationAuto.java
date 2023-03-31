package frc.robot.autos.drive;

import com.ctre.phoenixpro.Timestamp;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LevelChargingStationAuto extends CommandBase {

    private final Swerve swerve;
    private double timerTimeStamp;
    private double levelDuration = 2.0;

    public LevelChargingStationAuto(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.levelRobotPitch(); 
        timerTimeStamp = Timer.getFPGATimestamp();    
    }

    @Override
    public void execute() {
        swerve.levelRobotPitch();
        if (!swerve.isRobotLevel()) {
            timerTimeStamp = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - timerTimeStamp) > levelDuration;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 1, false, true);
    }
    
}
