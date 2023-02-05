package frc.robot;

import frc.robot.controls.DriverControls;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.controls.SecondaryDriveControls;

public class RobotContainer {
    private static Drivetrain drivetrain = new Drivetrain();
    private static AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(drivetrain);
    private static ArmSubsystem armSubsystem = new ArmSubsystem();
    private static ClawSubsystem clawSubsystem = new ClawSubsystem();
    private static DriverControls driverControls = new DriverControls(Constants.Controllers.DRIVER_CONTROLS_PORT, Constants.Controllers.CONTROL_PANEL_PORT);
    private static SecondaryDriveControls secondaryDriveControls = new SecondaryDriveControls(Constants.Controllers.SECONDARY_DRIVER_CONTROLS_PORT);

    private static boolean rotateAroundFront = false;

    public static void setRotateAroundFront(boolean rotateAroundFront) {
        RobotContainer.rotateAroundFront = rotateAroundFront;
    }

    public static ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public static boolean getRotateAroundFront() {
        return rotateAroundFront;
    }

    public static Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public static DriverControls getDriverControls() {
        return driverControls;
    }

    public static SecondaryDriveControls getSecondaryDriveControls() {
        return secondaryDriveControls;
    }

    public static AprilTagSubsystem getAprilTagSubsystem() {
        return aprilTagSubsystem;
    }

    public static ClawSubsystem getClawSubsystem() {
        return clawSubsystem;
    }
}
