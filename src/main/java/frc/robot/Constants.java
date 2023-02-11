package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*
 * This class is where the constants for the robot are stored. This is the sources of all modifyable values to tune the robot.
 */
public class Constants {

    public static final class Drive {
        public static final int kFrontLeftDriveMotorId = 6;
        public static final int kRearLeftDriveMotorId = 8;
        public static final int kFrontRightDriveMotorId = 7;
        public static final int kRearRightDriveMotorId = 1;

        public static final int kFrontLeftTurningMotorId = 5;
        public static final int kRearLeftTurningMotorId = 4;
        public static final int kFrontRightTurningMotorId = 3;
        public static final int kRearRightTurningMotorId = 2;

        public static final int kFrontLeftTurningEncoderId = 11;
        public static final int kRearLeftTurningEncoderId = 12;
        public static final int kFrontRightTurningEncoderId = 9;
        public static final int kRearRightTurningEncoderId = 10;

        public static final double kFrontLeftAngleZero = -130;
        public static final double kRearLeftAngleZero = -15;
        public static final double kFrontRightAngleZero = -171;
        public static final double kRearRightAngleZero = 40;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kRearRightDriveEncoderReversed = true;

        public static final int kPigeonId = 13;
    }

    public static final class Wheels {
        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double WHEEL_WIDTH_OFFSET = 0.168275;
        public static final double WHEEL_LENGTH_OFFSET = 0.282575;
        public static final double WHEEL_POLAR_DISTANCE = 0.32888;
    }

    public static final class Controllers {
        public static final int DRIVER_CONTROLS_PORT = 0;
        public static final int SECONDARY_DRIVER_CONTROLS_PORT = 1;
        public static final int CONTROL_PANEL_PORT = 2;
    }

    public static final class Claw {
        public static final int INTAKE_MOTOR_ID = 3;
        public static final int SOLINOID_FORWARD = 0;
        public static final int SOLINOID_REVERSE = 1;
        public static final double ALLOWED_ENCODER_ERROR = 5.0;
    }

    public static final class Arm {
        // The motor IDs for the motors in the CAN loop
        public static final int PRIMARY_MOTOR_ID = 14;
        public static final int SECONDARY_MOTOR_ID = 15;
        public static final int WRIST_MOTOR_ID = 2;

        // The IDs for the limit switches
        public static final int PRIMARY_LIMIT_SWITCH_FORWARD = 0;
        public static final int PRIMARY_LIMIT_SWITCH_REVERSE = 1;
        public static final int SECONDARY_LIMIT_SWITCH_FORWARD = 2;
        public static final int WRIST_LIMIT_SWITCH_FORWARD = 3;
        
        // The configuration for the arm motors
        public static final int MAX_CURRENT = 105;
        public static final int ENCODER_TICKS_PER_REVOLUTION = 42;
        public static final double MAX_RPM = 5000.0;

        // The measurements of the arm in meters
        public static final double PRIMARY_ARM_LENGTH = Units.inchesToMeters(34.0);
        public static final double SECONDARY_ARM_LENGTH = Units.inchesToMeters(34.0);
        public static final double CLAW_LENGTH = Units.inchesToMeters(16.0);

        // The gear ratios of the arm motors
        public static final double PRIMARY_ARM_GEAR_RATIO = 500.0;
        public static final double SECONDARY_ARM_GEAR_RATIO = 400.0;
        public static final double CLAW_GEAR_RATIO = 20.0;
        
        // These values are the maximum speed of the arm motors in percent output from 0 to 1
        public static final double PRIMARY_ARM_MAX_SPEED = 0.3;
        public static final double SECONDARY_ARM_MAX_SPEED = 0.3;

        // The maximum error in encoder ticks that the arm can be from the desired position
        public static final double ALLOWED_ENCODER_ERROR = 5.0;
    }

    // These values will be used to select predefined arm positions
    public static enum ArmPositions {
        BACK_POLE, FRONT_POLE, FLOOR_DROP, FLOOR_PICKUP, SHELF_PICKUP, PARK
    }

    public static final class RelativePositions {
        public static final Translation2d CLAW_PICKUP = new Translation2d(0.0, 1.0);
    }

    public static final class Camera {
        public static final double CAMERA_HEIGHT = 1.0;
        public static final double CAMERA_PITCH = 0.0;
    }

    // These values are all hardcoded for the specific position of the AprilTags on
    // the field including the rotation
    public static final class AprilTagPositions {
        public static final List<AprilTag> TAGS = List.of(null, new AprilTag(1,
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
}
