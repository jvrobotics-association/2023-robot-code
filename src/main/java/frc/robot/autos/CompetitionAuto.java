package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Swerve;

public class CompetitionAuto extends SequentialCommandGroup {
    public CompetitionAuto(Swerve swerve, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, boolean isRed){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory =
            isRed ? TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.inchesToMeters((54*12+3.25)-84.1), Units.inchesToMeters(194.7), new Rotation2d(180)),
                // Make the 3 other points
                List.of(new Translation2d(Units.inchesToMeters(20+17.5) + Constants.AutoConstants.kRedChargingStationX, Units.inchesToMeters(12)+Constants.AutoConstants.kRedChargingStationY)),
                // End at the origin
                new Pose2d(Constants.AutoConstants.kRedChargingStationX, Units.inchesToMeters(12)+Constants.AutoConstants.kRedChargingStationY, new Rotation2d(0)),
                config) : TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(Units.inchesToMeters(84.1), Units.inchesToMeters(194.7), new Rotation2d(0)),
                    // Make the 3 other points
                    List.of(new Translation2d((-Units.inchesToMeters(20+17.5)) + Constants.AutoConstants.kBlueChargingStationX, Units.inchesToMeters(12)+Constants.AutoConstants.kBlueChargingStationY)),
                    // End at the origin
                    new Pose2d(Constants.AutoConstants.kBlueChargingStationX, Units.inchesToMeters(12)+Constants.AutoConstants.kBlueChargingStationY, new Rotation2d(180)),
                    config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            new CalibrateArmCommand(armSubsystem, clawSubsystem),
            new MoveToPresetArmPosition(armSubsystem, clawSubsystem, ArmPositions.BACK_POLE),
            new ReverseClawIntakeCommand(clawSubsystem),
            new CalibrateArmCommand(armSubsystem, clawSubsystem)
            // swerveControllerCommand,
            // new LevelChargingStationAuto(swerve)
        );
    }
}