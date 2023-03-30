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
import frc.robot.commands.april_tag.UpdateRobotPositionCommand;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.arm.HoldArmPositionCommand;
import frc.robot.commands.arm.MoveArmBackward;
import frc.robot.commands.arm.MoveArmForward;
import frc.robot.commands.claw.MoveWristDownCommand;
import frc.robot.commands.claw.MoveWristUpCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.claw.ReverseClawIntakeFastCommand;
import frc.robot.commands.claw.RunClawIntakeCommand;
import frc.robot.commands.combined.MoveToPresetArmPosition;
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
    // private final Joystick driver = new Joystick(0);
    private final Joystick driver = new Joystick(0);
    private final Joystick control = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = 2;

    private boolean isRobotCentric = false;
    private boolean isSlowDrive = false;

    private final BooleanSupplier isRobotCentricSupplier = () -> isRobotCentric;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 9);
    private final JoystickButton robotCentric = new JoystickButton(driver, 7);
    private final JoystickButton fieldCentric = new JoystickButton(driver, 8);
    private final JoystickButton slowDriveMode = new JoystickButton(driver, 2);
    // private final JoystickButton levelRobot = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton alignRobot = new JoystickButton(control, 2);
    // private final JoystickButton alignRobotToAprilTag = new JoystickButton(control, 3);
    // private final JoystickButton zeroOdometry = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Arm Buttons */
    private final JoystickButton armForward = new JoystickButton(control, 12);
    private final JoystickButton armBackward = new JoystickButton(control, 13);    

    // /* Claw Buttons */
    private final JoystickButton moveWristUp = new JoystickButton(control, 10);
    private final JoystickButton moveWristDown = new JoystickButton(control, 11);
    private final JoystickButton runIntakeFoward = new JoystickButton(control, 7);
    private final JoystickButton runIntakeReverse = new JoystickButton(control, 8);
    private final JoystickButton runIntakeReverseFast = new JoystickButton(control, 9);

    // /* Preset Position Buttons */
    private final JoystickButton frontPole = new JoystickButton(control, 6);
    private final JoystickButton backShelf = new JoystickButton(control, 4);
    private final JoystickButton frontShelf = new JoystickButton(control, 5);
    private final JoystickButton floorDrop = new JoystickButton(control, 1);
    private final JoystickButton sliderPickup = new JoystickButton(control, 2);
    private final JoystickButton calibrateArm = new JoystickButton(control, 3);

    /* April Tag Buttons */
    private final JoystickButton calculateRobotPosition = new JoystickButton(driver, 12);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final GrabberSubsystem s_Grabber = new GrabberSubsystem();
    private final AprilTagSubsystem s_AprilTag = new AprilTagSubsystem(s_Swerve);
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();

    /* Autonomous */
    // Moves in a S shape
    // private final Command exampleAuto = new ExampleAuto(s_Swerve);

    // Moves in a diamond shape
    // private final Command diamondAuto = new DiamondAuto(s_Swerve, s_Grabber);
    // private final Command competitionAuto = new CompetitionAuto(s_Swerve, s_Grabber, isRed);
    // private final Command simpleAutoMiddle = new VerySimpleAuto(s_Swerve, s_Grabber, isRed, ArmPositions.FRONT_POLE);
    // private final Command simpleAutoBottom = new VerySimpleAuto(s_Swerve, s_Grabber, isRed, ArmPositions.FLOOR_DROP);

    // A chooser for autonomous commands
    private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> isRobotCentricSupplier.getAsBoolean(),
                        () -> isYLocked,
                        () -> slowDriveMode.getAsBoolean()));

        s_Grabber.setDefaultCommand(new HoldArmPositionCommand(s_Grabber));

        // Configure the button bindings
        configureButtonBindings();

        // Add commands to the autonomous command chooser
        // autonomousChooser.setDefaultOption("Competition Auto", competitionAuto);
        // autonomousChooser.addOption("Example Auto", exampleAuto);
        // autonomousChooser.addOption("Diamond Auto", diamondAuto);
        // autonomousChooser.setDefaultOption("Simple Auto Middle", simpleAutoMiddle);
        // autonomousChooser.addOption("Simple Auto Bottom", simpleAutoBottom);
        autonomousChooser.setDefaultOption("Competition Auto", new BottomLMoveOutOfCommunityAuto(s_Swerve, s_Grabber, s_Intake, isRed));
        autonomousChooser.addOption("Do Nothing", new InstantCommand());
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
        // levelRobot.whileTrue(new LevelChargingStationAuto(s_Swerve));
        // alignRobot.whileTrue(new StraightenRobot(s_Swerve, this, () -> -driver.getRawAxis(strafeAxis)));
        // alignRobotToAprilTag.whileTrue(new AlignToAprilTag(s_Swerve));
        // zeroOdometry.onTrue(new ZeroOdometry(s_Swerve));

        /* Arm Buttons */
        armForward.whileTrue(new MoveArmForward(s_Grabber));
        armBackward.whileTrue(new MoveArmBackward(s_Grabber));
        calibrateArm.onTrue(new CalibrateArmCommand(s_Grabber));

        /* Claw Buttons */
        moveWristUp.whileTrue(new MoveWristUpCommand(s_Grabber));
        moveWristDown.whileTrue(new MoveWristDownCommand(s_Grabber));
        runIntakeFoward.whileTrue(new RunClawIntakeCommand(s_Intake));
        runIntakeReverse.whileTrue(new ReverseClawIntakeCommand(s_Intake, s_Swerve));
        runIntakeReverseFast.whileTrue(new ReverseClawIntakeFastCommand(s_Intake));

        // /* Preset Position Buttons */
        frontPole.onTrue(new MoveToPresetArmPosition(s_Grabber, ArmPositions.FRONT_POLE));
        backShelf.onTrue(new MoveToPresetArmPosition(s_Grabber, ArmPositions.BACK_SHELF));
        frontShelf.onTrue(new MoveToPresetArmPosition(s_Grabber, ArmPositions.FRONT_SHELF));
        floorDrop.onTrue(new MoveToPresetArmPosition(s_Grabber, ArmPositions.FLOOR_DROP));
        sliderPickup.onTrue(new MoveToPresetArmPosition(s_Grabber, ArmPositions.SLIDER_PICKUP));

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
        // Command selection = new NoArmAuto(s_Swerve, s_Grabber, s_Intake, isRed);
        if (selection == null) {
            return new BottomLMoveOutOfCommunityAuto(s_Swerve, s_Grabber, s_Intake, isRed);
        }
        // System.out.println(selection.getName());
        return selection;
    }

    /*
     * Getters for subsystems
     */
    public Swerve getSwerve() {
        return s_Swerve;
    }

    public GrabberSubsystem getGrabber() {
        return s_Grabber;
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
