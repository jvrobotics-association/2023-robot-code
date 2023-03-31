package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean isRed = true;

    public Swerve() {
        isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro(180.0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public double getPitch() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Pitch", ypr[2]);
        return ypr[2];
    }



    public void levelRobotPitch() {
        double pitch = getPitch();

        // check to see if the robot is going to fall off the table
        // Translation2d robotPos = swerveOdometry.getPoseMeters().getTranslation();
        // if (isRed) {
        // if (Math.abs(Constants.AutoConstants.kRedChargingStationX - robotPos.getX())
        // > AutoConstants.kAllowedChargingStationMovementFromCenter) {
        // drive(new Translation2d(0, 0), 0, false, true);
        // return;
        // }
        // } else {
        // if (Math.abs(Constants.AutoConstants.kBlueChargingStationX - robotPos.getX())
        // > AutoConstants.kAllowedChargingStationMovementFromCenter) {
        // drive(new Translation2d(0, 0), 0, false, true);
        // return;
        // }
        // }

        double expectedMovement = 0;

        if (pitch > AutoConstants.kRobotPitchTolerance) {

            expectedMovement = pitch / 180.0;
        } else if (pitch < -AutoConstants.kRobotPitchTolerance) {
            expectedMovement = pitch / 180.0;
        }

        SmartDashboard.putNumber("Level Expected Movement", -expectedMovement);
        drive(new Translation2d(-expectedMovement, 0), 0, false, true);
    }

    public boolean isRobotLevel() {
        return Math.abs(getPitch()) < AutoConstants.kRobotPitchTolerance;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void zeroGyro(double value) {
        gyro.setYaw(value);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void spinRobotToTarget(double angle, double strafe, double translation) {
        int direction = (int) Math.signum(angle - getYaw().getDegrees());
        if (Math.abs(angle - getYaw().getDegrees()) < Constants.Swerve.gyroDeadZone
                || Math.abs(angle + 180 - ((getYaw().getDegrees() + 180) % 360)) < Constants.Swerve.gyroDeadZone) {
            drive(new Translation2d(translation * Constants.Swerve.maxSpeed, strafe * Constants.Swerve.maxSpeed), 0, false, true);
        } else {
            drive(new Translation2d(translation * Constants.Swerve.maxSpeed, strafe * Constants.Swerve.maxSpeed), direction * Constants.Swerve.maxAngularVelocity , false,
                    true);
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());

        // for(SwerveModule mod : mSwerveMods){
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
        // mod.getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
        // mod.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
        // mod.getState().speedMetersPerSecond);
        // }

        SmartDashboard.putString("Calculated Robot Position", swerveOdometry.getPoseMeters().toString());
    }
}