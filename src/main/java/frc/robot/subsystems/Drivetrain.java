
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

        // The gyro sensor
        public final PigeonIMU m_gyro = new PigeonIMU(13);

        private int gyroOffset = 0;

        private double l = Constants.Wheels.WHEEL_LENGTH_OFFSET;
        private double w = Constants.Wheels.WHEEL_WIDTH_OFFSET;
        private double r = Constants.Wheels.WHEEL_POLAR_DISTANCE;

        private final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(l, w),
                        new Translation2d(l, -w),
                        new Translation2d(-l, w),
                        new Translation2d(-l, -w));

        private final SwerveModulePosition[] swerveModulePositions = {
                        new SwerveModulePosition(r, new Rotation2d(l, w)),
                        new SwerveModulePosition(r, new Rotation2d(l, -w)),
                        new SwerveModulePosition(r, new Rotation2d(-l, w)),
                        new SwerveModulePosition(r, new Rotation2d(-l, -w)), };

        // Odometry class for tracking robot pose
        public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics,
                        new Rotation2d(m_gyro.getRoll()), swerveModulePositions);
        private final ShuffleboardTab moduleTab = Shuffleboard.getTab("Module Info");
        private final SwerveModule m_frontLeft = new SwerveModule(
                        Constants.Drive.kFrontLeftDriveMotorId,
                        Constants.Drive.kFrontLeftTurningMotorId,
                        Constants.Drive.kFrontLeftTurningEncoderId,
                        Constants.Drive.kFrontLeftAngleZero,
                        Constants.Drive.kFrontLeftTurningEncoderReversed,
                        Constants.Drive.kFrontLeftDriveEncoderReversed,
                        moduleTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                        .withSize(4, 8)
                                        .withPosition(0, 0));
        private final SwerveModule m_rearLeft = new SwerveModule(
                        Constants.Drive.kRearLeftDriveMotorId,
                        Constants.Drive.kRearLeftTurningMotorId,
                        Constants.Drive.kRearLeftTurningEncoderId,
                        Constants.Drive.kRearLeftAngleZero,
                        Constants.Drive.kRearLeftTurningEncoderReversed,
                        Constants.Drive.kRearLeftDriveEncoderReversed,
                        moduleTab.getLayout("Rear Left Module", BuiltInLayouts.kList)
                                        .withSize(4, 8)
                                        .withPosition(4, 0));
        private final SwerveModule m_frontRight = new SwerveModule(
                        Constants.Drive.kFrontRightDriveMotorId,
                        Constants.Drive.kFrontRightTurningMotorId,
                        Constants.Drive.kFrontRightTurningEncoderId,
                        Constants.Drive.kFrontRightAngleZero,
                        Constants.Drive.kFrontRightTurningEncoderReversed,
                        Constants.Drive.kFrontRightDriveEncoderReversed,
                        moduleTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(4, 8)
                                        .withPosition(8, 0));
        private final SwerveModule m_rearRight = new SwerveModule(
                        Constants.Drive.kRearRightDriveMotorId,
                        Constants.Drive.kRearRightTurningMotorId,
                        Constants.Drive.kRearRightTurningEncoderId,
                        Constants.Drive.kRearRightAngleZero,
                        Constants.Drive.kRearRightTurningEncoderReversed,
                        Constants.Drive.kRearRightDriveEncoderReversed,
                        moduleTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
                                        .withSize(4, 8)
                                        .withPosition(12, 0));

        /**
         * Creates a new DriveSubsystem.
         */
        public Drivetrain() {
        }

        public double speed() {
                return 0;
        }

        @Override
        public void periodic() {
                // Update the odometry in the periodic block
                m_odometry.update(
                                new Rotation2d(m_gyro.getYaw()),
                                swerveModulePositions);

                // SmartDashboard.putString("m_frontLeft", m_frontLeft.getState().toString());
                // SmartDashboard.putString("m_rearLeft", m_rearLeft.getState().toString());
                // SmartDashboard.putString("m_frontRight", m_frontRight.getState().toString());
                // SmartDashboard.putString("m_rearRight", m_rearRight.getState().toString());
                // SmartDashboard.putString("odometry", m_odometry.getPoseMeters().toString());
                // SmartDashboard.putString("rotation2d", m_gyro.getRotation2d().toString());
                // SmartDashboard.putNumber("angle", m_gyro.getAngle());
                // m_frontLeft.periodic_func();
                // m_rearRight.periodic_func();
                // m_rearLeft.periodic_func();
                // m_frontRight.periodic_func();
        }

        public double heading() {
                return (m_gyro.getYaw() + this.gyroOffset) % 360;
        }

        public void setGyroOffset(int gyroOffset) {
                // this.gyroOffset = gyroOffset;
        }

        public double getPitch() {
                return (double) m_gyro.getPitch();
        }

        public void spinDriveMotors(double speed) {
                m_frontLeft.setDriveMotor(speed);
                m_rearLeft.setDriveMotor(speed);
                m_frontRight.setDriveMotor(speed);
                m_rearRight.setDriveMotor(speed);
        }

        public void rotateMotorsForward () {
                m_frontLeft.rotateWheelTo(0);
                m_rearLeft.rotateWheelTo(0);
                m_frontRight.rotateWheelTo(0);
                m_rearRight.rotateWheelTo(0);
        }

        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(new Rotation2d(m_gyro.getYaw()), swerveModulePositions, pose);
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        @SuppressWarnings("ParameterName")
        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
                        Translation2d rotationOffset) {
                SmartDashboard.putBoolean("Field Relative:", fieldRelative);
                SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                Rotation2d.fromDegrees(getHeading()))
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                swerveModuleStates, 4.5);
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_rearLeft.setDesiredState(swerveModuleStates[2]);
                m_rearRight.setDesiredState(swerveModuleStates[3]);
                // SmartDashboard.putString("Front Left desired state: ",
                // swerveModuleStates[0].toString());
                // SmartDashboard.putString("Front Right desired state: ",
                // swerveModuleStates[1].toString());
                m_frontLeft.periodic_func();
                m_frontRight.periodic_func();
                m_rearLeft.periodic_func();
                m_rearRight.periodic_func();
        }

        /**
         * Sets the swerve ModuleStates.
         *
         * @param desiredStates The desired SwerveModule states.
         */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                desiredStates, 4.5);
                m_frontLeft.setDesiredState(desiredStates[0]);
                m_frontRight.setDesiredState(desiredStates[1]);
                m_rearLeft.setDesiredState(desiredStates[2]);
                m_rearRight.setDesiredState(desiredStates[3]);
        }

        /**
         * Zeroes the heading of the robot.
         */
        public void zeroHeading() {
                gyroOffset = 0;
                m_gyro.setYaw(0);
        }

        /**
         * Returns the heading of the robot.
         *
         * @return the robot's heading in degrees, from -180 to 180
         */
        public double getHeading() {
                double currentHeading = m_gyro.getYaw() + gyroOffset;
                if (currentHeading > 180) {
                        currentHeading -= 360;
                } else if (currentHeading < -180) {
                        currentHeading += 360;
                }
                return currentHeading;
        }

        // /**
        // * Returns the turn rate of the robot.
        // *
        // * @return The turn rate of the robot, in degrees per second
        // */
        // public double getTurnRate() {
        //// return m_gyro.getRate() * (Constants.Drive.kGyroReversed ? -1.0 : 1.0);
        // return m_gyro.getRate();
        // }

}
