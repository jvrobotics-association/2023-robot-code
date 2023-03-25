package frc.robot.autos.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class MoveRobotAuto extends CommandBase {

    private final Swerve swerve;
    private double timestamp;
    private final Translation2d movement;
    private final double duration;

    public MoveRobotAuto(Swerve swerve, Translation2d movement, double duration) {
        this.swerve = swerve;
        this.movement = movement;
        this.duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        swerve.drive(movement, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timestamp > duration;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }
    
}
