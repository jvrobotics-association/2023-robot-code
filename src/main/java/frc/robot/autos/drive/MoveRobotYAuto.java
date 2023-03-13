package frc.robot.autos.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class MoveRobotYAuto extends CommandBase {

    private final Swerve swerve;
    private double timestamp;
    private final int direction;
    private final double duration;

    public MoveRobotYAuto(Swerve swerve, int direction, double duration) {
        this.swerve = swerve;
        this.direction = direction;
        this.duration = duration;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(180)));
        timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0, -0.2 * direction), 0, true, true);
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
