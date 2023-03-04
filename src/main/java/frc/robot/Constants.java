


package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Claw {
        public static final int wristMotorId = 16;
        public static final int intakeMotorId = 17;
        public static final int wristLimitSwitchUpId = 8;
        public static final int wristLimitSwitchDownId = 9;
        public static final double wristGearRatio = 20.0;
        public static final double clawLength = Units.inchesToMeters(16.0);
        public static final double wristMotorSpeed = 0.2;
        public static final double wristMotorTolerance = 5.0;
    }

    public static enum ArmPositions {

        BACK_POLE(61.9, -181.6, 38.6),
        FRONT_POLE(14.7, -87.6, 30.9),
        BACK_SHELF(16.7, -63.5, 29.1),
        FRONT_SHELF(66.5, -165.2, 44.6),
        FLOOR_DROP(63.5, -27.8, 16.6),
        FLOOR_PICKUP_TOP(40.1, -32.7, 42.0),
        STARTING_POSITION(0.0, 0.0, 0.0),
        SLIDER_PICKUP(0.0, -82.2, 36.0),
        // TODO: find the correct values for these
        KNOWN_GOOD_CONFIGURATION(35.5, -50.0, 20.0);

        private final double primaryArmAngle;
        private final double secondaryArmAngle;
        private final double wristAngle;

        ArmPositions(double primaryArmAngle, double secondaryArmAngle, double wristAngle) {
            this.primaryArmAngle = primaryArmAngle;
            this.secondaryArmAngle = secondaryArmAngle;
            this.wristAngle = wristAngle;
        }

        public double getPrimaryArmAngle() {
            return primaryArmAngle;
        }

        public double getSecondaryArmAngle() {
            return secondaryArmAngle;
        }

        public double getWristAngle() {
            return wristAngle;
        }

        // get both arm angles in a list
        public List<Double> getArmAngles() {
            return List.of(primaryArmAngle, secondaryArmAngle, wristAngle);
        }

    }

    public static final class Arm {
        public static final int primaryArmMotorId = 14;
        public static final int secondaryArmMotorId = 15;
        public static final int primaryLimitSwitchForwardId = 0;
        public static final int primaryLimitSwitchReverseId = 1;
        public static final int secondaryLimitSwitchUpId = 2;
        public static final int secondaryLimitSwitchDownId = 3;
        public static final int encoderTicksPerRevolution = 42;
        public static final double allowedEncoderError = 1.0;

        // These values are in percent output (0.0 to 1.0)
        public static final double primaryArmMaxSpeed = 0.3;
        public static final double secondaryArmMaxSpeed = 0.5;

        public static final double primaryArmLength = Units.inchesToMeters(36.0);
        public static final double secondaryArmLength = Units.inchesToMeters(30.0);

        // The gear ratios of the arm motors
        public static final double primaryArmGearRatio = 500.0;
        public static final double secondaryArmGearRatio = 400.0;

        public static final double primaryArmEncoderZero = 0;
        public static final double secondaryArmEncoderZero = 0;

        public static final double zeroAreaPrimaryEncoderValue = 20;
        public static final double zeroAreaSecondaryEncoderValue = -70;
    }

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SwerveXFlipped(
                COTSFalconSwerveConstants.driveGearRatios.SWERVEX_FLIPPED_Gears_L2,
                COTSFalconSwerveConstants.angleGearRatios.SWERVEX_FLIPPED_GEARS);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(13.25);
        public static final double wheelBase = Units.inchesToMeters(22.25);
        public static final double gyroDeadZone = 3.0;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 0.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 0.25; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(328.97);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.44);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(220.63);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(188.20);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kRobotPitchTolerance = 1.0;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // width is the x dimension, length is the y dimension
        public static final double kChargingStationWidth = Units.feetToMeters(4.0);
        public static final double kChargingStationLength = Units.feetToMeters(8.0);

        // note this is the distance between the charging station center and the origin of the field coordinate system
        public static final double kRedChargingStationX = Units.inchesToMeters(60.00) + kChargingStationLength/2;
        public static final double kRedChargingStationY = Units.inchesToMeters(499.00);
        public static final double kBlueChargingStationX = Units.inchesToMeters(60.00) + kChargingStationLength/2;
        public static final double kBlueChargingStationY = Units.inchesToMeters(154.00);

        public static final double kRobotPitchMovement = 0.07; // this is the distance the robot moves in the x direction when it pitches        

        // this measurement is in meters and is the allowed distance the robot can travel from the center of the charging station to try to level itself
        public static final double kAllowedChargingStationMovementFromCenter = 0.25;


        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AprilTagPositions {
        public static final List<AprilTag> tags = List.of(new AprilTag(0, new Pose3d(0, 0, 0, new Rotation3d())),
                new AprilTag(1,
                        new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d())),
                new AprilTag(2,
                        new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d())),
                new AprilTag(3,
                        new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d())),
                new AprilTag(4,
                        new Pose3d(16.178784, 6.749796, 0.695452, new Rotation3d())),
                new AprilTag(5,
                        new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(0, 0, 180))),
                new AprilTag(6,
                        new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(0, 0, 180))),
                new AprilTag(7,
                        new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, 180))),
                new AprilTag(8,
                        new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, 180))));
    }

    public static final class Camera {
        public static final double cameraHeight = Units.inchesToMeters(22 + 3.0/16.0);
        public static final double cameraXOffset = Units.inchesToMeters(10.25);
        public static final double cameraYOffset = Units.inchesToMeters(2.75);
        public static final double cameraPitch = 0.0;
    }
}
