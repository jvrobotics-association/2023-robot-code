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

    public static final int kFrontLeftDriveMotorPort = 18;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 23;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 25;

    public static final int kFrontLeftTurningEncoderPort = 22;
    public static final int kRearLeftTurningEncoderPort = 10;
    public static final int kFrontRightTurningEncoderPort = 9;
    public static final int kRearRightTurningEncoderPort = 8;

    public static final double kFrontLeftAngleZero = 79.45;
    public static final double kRearLeftAngleZero = 121.38;
    public static final double kFrontRightAngleZero = -104.68;
    public static final double kRearRightAngleZero = 23.54;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

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
        public static final int INTAKE_MOTOR = 3;
        public static final int SOLINOID_FORWARD = 0;
        public static final int SOLINOID_REVERSE = 1;
        public static final double ALLOWED_ENCODER_ERROR = 5.0;
    }

    public static final class Arm {
        public static final int PRIMARY_MOTOR = 0;
        public static final int SECONDARY_MOTOR = 1;
        public static final int WRIST_MOTOR = 2;
        public static final int MAX_CURRENT = 105;
        public static final int ENCODER_TICKS_PER_REVOLUTION = 42;
        public static final double MAX_RPM = 5000.0;
        public static final double PRIMARY_ARM_LENGTH = Units.inchesToMeters(34.0);
        public static final double SECONDARY_ARM_LENGTH = Units.inchesToMeters(34.0);
        public static final double CLAW_LENGTH = Units.inchesToMeters(16.0);
        public static final double PRIMARY_ARM_GEAR_RATIO = 500.0;
        public static final double SECONDARY_ARM_GEAR_RATIO = 400.0;
        public static final double CLAW_GEAR_RATIO = 20.0;
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

    // These values are all hardcoded for the specific position of the AprilTags on the field including the rotation
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
