package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DiamondAuto extends SequentialCommandGroup {
    public DiamondAuto(Swerve swerve, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Make the 3 other points
                List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(1, -1)),
                // End at the origin
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            Commands.parallel(swerveControllerCommand, new MoveToPresetArmPosition(armSubsystem, clawSubsystem, ArmPositions.BACK_POLE)),
            new ReverseClawIntakeCommand(clawSubsystem),
            new MoveToPresetArmPosition(armSubsystem, clawSubsystem, ArmPositions.STARTING_POSITION)
            
        );
    }
}