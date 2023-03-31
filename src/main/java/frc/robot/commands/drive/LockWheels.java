package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LockWheels extends CommandBase {
    
    private final Swerve swerve;
    private double timerTimeStamp;
    private double levelDuration = 0.5;

    public LockWheels(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timerTimeStamp = Timer.getFPGATimestamp();    
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
