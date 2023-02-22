package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.april_tag.UpdateRobotPositionCommand;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.commands.arm.InverseKinematicsCommand;
import frc.robot.commands.arm.MovePrimaryArmBackwardsCommand;
import frc.robot.commands.arm.MovePrimaryArmForwardCommand;
import frc.robot.commands.arm.MoveSecondaryArmDownCommand;
import frc.robot.commands.arm.MoveSecondaryArmUpCommand;
import frc.robot.commands.claw.MoveWristDownCommand;
import frc.robot.commands.claw.MoveWristUpCommand;
import frc.robot.commands.claw.ReverseClawIntakeCommand;
import frc.robot.commands.claw.RunClawIntakeCommand;
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
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick control = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton fieldCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Arm Buttons */
    private final JoystickButton primaryArmForward = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton primaryArmReverse = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton secondaryArmUp = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton secondaryArmDown = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton inverseArmKinematics = new JoystickButton(operator, 3);
    private final JoystickButton calibrateArm = new JoystickButton(operator, 4);

    /* Claw Buttons */
    private final JoystickButton moveWristUp = new JoystickButton(operator, 7);
    private final JoystickButton moveWristDown = new JoystickButton(operator, 9);
    private final JoystickButton runIntakeFoward = new JoystickButton(operator, 8);
    private final JoystickButton runIntakeReverse = new JoystickButton(operator, 10);

    /* April Tag Buttons */
    private final JoystickButton calculateRobotPosition = new JoystickButton(control, 1);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final ClawSubsystem s_Claw = new ClawSubsystem();
    private final AprilTagSubsystem s_AprilTag = new AprilTagSubsystem(s_Swerve);

    /* Autonomous */
    // Moves in a S shape
    private final Command exampleAuto = new ExampleAuto(s_Swerve);

    // Moves in a diamond shape
    private final Command diamondAuto = new DiamondAuto(s_Swerve);

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
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();

        // Add commands to the autonomous command chooser
        autonomousChooser.setDefaultOption("Example Auto", exampleAuto);
        autonomousChooser.addOption("Diamond Auto", diamondAuto);
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

        /* Arm Buttons */
        primaryArmForward.whileTrue(new MovePrimaryArmForwardCommand(s_Arm));
        primaryArmReverse.whileTrue(new MovePrimaryArmBackwardsCommand(s_Arm));
        secondaryArmUp.whileTrue(new MoveSecondaryArmUpCommand(s_Arm));
        secondaryArmDown.whileTrue(new MoveSecondaryArmDownCommand(s_Arm));
        inverseArmKinematics.whileTrue(new InverseKinematicsCommand(s_Arm));
        calibrateArm.whileTrue(new CalibrateArmCommand(s_Arm));

        /* Claw Buttons */
        moveWristUp.whileTrue(new MoveWristUpCommand(s_Claw));
        moveWristDown.whileTrue(new MoveWristDownCommand(s_Claw));
        runIntakeFoward.whileTrue(new RunClawIntakeCommand(s_Claw));
        runIntakeReverse.whileTrue(new ReverseClawIntakeCommand(s_Claw));

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
}
