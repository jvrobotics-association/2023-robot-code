package frc.robot.autos.drive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToAprilTag extends CommandBase {

    private Swerve swerve;

    private TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

    private Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(1, -1)),
        new Pose2d(0, 0, new Rotation2d(0)),
        config
    );

    private ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    private SwerveControllerCommand swerveControllerCommand;
        

    public AlignToAprilTag(Swerve swerve) {
        this.swerve = swerve;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        new Rotation2d();
        // set the new trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(
            swerve.getPose(),
            List.of(),
            new Pose2d(18.0, 4.14, Rotation2d.fromDegrees(180)),
            config
        );

        // reset the swerve controller command
        swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

        swerveControllerCommand.initialize();
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }
    
}
