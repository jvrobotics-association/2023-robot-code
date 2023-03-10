package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmPositions;
import frc.robot.autos.*;
import frc.robot.autos.drive.LevelChargingStationAuto;
import frc.robot.commands.april_tag.UpdateRobotPositionCommand;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.arm.MovePrimaryArmBackwardsCommand;
import frc.robot.commands.arm.MovePrimaryArmForwardCommand;
import frc.robot.commands.arm.MoveSecondaryArmDownCommand;
import frc.robot.commands.arm.MoveSecondaryArmUpCommand;
import frc.robot.commands.claw.MoveWristDownCommand;
import frc.robot.commands.claw.MoveWristUpCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.claw.RunClawIntakeCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
import frc.robot.commands.drive.StraightenRobot;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;
    private boolean isYLocked = false;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick control = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private boolean isRobotCentric = true;

    private final BooleanSupplier isRobotCentricSupplier = () -> isRobotCentric;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton fieldCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton levelRobot = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton alignRobot = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Arm Buttons */
    private final JoystickButton primaryArmForward = new JoystickButton(control, 4);
    private final JoystickButton primaryArmReverse = new JoystickButton(control, 5);
    private final JoystickButton secondaryArmUp = new JoystickButton(control, 6);
    private final JoystickButton secondaryArmDown = new JoystickButton(control, 7);
    // private final JoystickButton inverseArmKinematics = new JoystickButton(operator, 3);
    // private final JoystickButton calibrateArm = new JoystickButton(operator, 4);

    /* Claw Buttons */
    private final JoystickButton moveWristUp = new JoystickButton(operator, 5);
    private final JoystickButton moveWristDown = new JoystickButton(operator, 3);
    private final JoystickButton runIntakeFoward = new JoystickButton(operator, 1);
    private final JoystickButton runIntakeReverse = new JoystickButton(operator, 2);

    /* Preset Position Buttons */
    private final JoystickButton backPole = new JoystickButton(operator, 7);
    private final JoystickButton frontPole = new JoystickButton(operator, 9);
    private final JoystickButton backShelf = new JoystickButton(operator, 10);
    private final JoystickButton frontShelf = new JoystickButton(operator, 8);
    private final JoystickButton floorDrop = new JoystickButton(operator, 11);
    private final JoystickButton floorPickupTop = new JoystickButton(operator, 12);
    private final JoystickButton startingPosition = new JoystickButton(operator, 6);
    private final JoystickButton sliderPickup = new JoystickButton(operator, 4);

    /* April Tag Buttons */
    private final JoystickButton calculateRobotPosition = new JoystickButton(control, 1);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final ClawSubsystem s_Claw = new ClawSubsystem();
    private final AprilTagSubsystem s_AprilTag = new AprilTagSubsystem(s_Swerve);

    /* Autonomous */
    // Moves in a S shape
    // private final Command exampleAuto = new ExampleAuto(s_Swerve);

    // Moves in a diamond shape
    // private final Command diamondAuto = new DiamondAuto(s_Swerve, s_Arm, s_Claw);
    // private final Command competitionAuto = new CompetitionAuto(s_Swerve, s_Arm, s_Claw, isRed);
    private final Command simpleAutoMiddle = new VerySimpleAuto(s_Swerve, s_Arm, s_Claw, isRed, ArmPositions.FRONT_POLE);
    private final Command simpleAutoBottom = new VerySimpleAuto(s_Swerve, s_Arm, s_Claw, isRed, ArmPositions.FLOOR_DROP);

    // A chooser for autonomous commands
    SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> isRobotCentricSupplier.getAsBoolean(),
                        () -> isYLocked));

        // Configure the button bindings
        configureButtonBindings();

        // Add commands to the autonomous command chooser
        // autonomousChooser.setDefaultOption("Competition Auto", competitionAuto);
        // autonomousChooser.addOption("Example Auto", exampleAuto);
        // autonomousChooser.addOption("Diamond Auto", diamondAuto);
        autonomousChooser.setDefaultOption("Simple Auto Middle", simpleAutoMiddle);
        autonomousChooser.addOption("Simple Auto Bottom", simpleAutoBottom);
        SmartDashboard.putData("Autonomous Chooser", autonomousChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        robotCentric.onTrue(new InstantCommand(() -> isRobotCentric = true));
        fieldCentric.onTrue(new InstantCommand(() -> isRobotCentric = false));
        levelRobot.whileTrue(new LevelChargingStationAuto(s_Swerve));
        alignRobot.whileTrue(new StraightenRobot(s_Swerve, this, () -> -driver.getRawAxis(strafeAxis)));

        /* Arm Buttons */
        primaryArmForward.whileTrue(new MovePrimaryArmForwardCommand(s_Arm));
        primaryArmReverse.whileTrue(new MovePrimaryArmBackwardsCommand(s_Arm));
        secondaryArmUp.whileTrue(new MoveSecondaryArmUpCommand(s_Arm));
        secondaryArmDown.whileTrue(new MoveSecondaryArmDownCommand(s_Arm));
        // inverseArmKinematics.whileTrue(new InverseKinematicsCommand(s_Arm));
        // calibrateArm.whileTrue(new CalibrateArmCommand(s_Arm, s_Claw));

        /* Claw Buttons */
        moveWristUp.whileTrue(new MoveWristUpCommand(s_Claw));
        moveWristDown.whileTrue(new MoveWristDownCommand(s_Claw));
        runIntakeFoward.whileTrue(new RunClawIntakeCommand(s_Claw));
        runIntakeReverse.whileTrue(new ReverseClawIntakeCommand(s_Claw));

        /* Preset Position Buttons */
        backPole.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.BACK_POLE));
        frontPole.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.FRONT_POLE));
        backShelf.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.BACK_SHELF));
        frontShelf.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.FRONT_SHELF));
        floorDrop.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.FLOOR_DROP));
        floorPickupTop.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.FLOOR_PICKUP_TOP));
        startingPosition.onTrue(new CalibrateArmCommand(s_Arm, s_Claw));
        sliderPickup.onTrue(new MoveToPresetArmPosition(s_Arm, s_Claw, ArmPositions.SLIDER_PICKUP));

        /* April Tag Buttons */
        calculateRobotPosition.whileTrue(new UpdateRobotPositionCommand(s_AprilTag));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        Command selection = autonomousChooser.getSelected();
        // System.out.println(selection.getName());
        return selection;
    }

    /*
     * Getters for subsystems
     */
    public Swerve getSwerve() {
        return s_Swerve;
    }

    public ArmSubsystem getArm() {
        return s_Arm;
    }

    public ClawSubsystem getClaw() {
        return s_Claw;
    }

    public AprilTagSubsystem getAprilTag() {
        return s_AprilTag;
    }

    public boolean getYLock() {
        return isYLocked;
    }

    public void setYLock(boolean value) {
        isYLocked = value;
    }
}
