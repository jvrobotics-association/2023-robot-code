package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends CommandBase {
    private final Drivetrain drivetrain = RobotContainer.getDrivetrain();
    private final DriverControls driverControls = RobotContainer.getDriverControls();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final double forwardDeadband = 0.07;
    private final double strafeDeadband = 0.07;
    private final double yawDeadband = 0.07;

    public TeleopDriveCommand() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverControls.getForward(), forwardDeadband))
                * 4.5;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverControls.getStrafe(), strafeDeadband))
                * 4.5;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverControls.getYaw(), yawDeadband))
                * 3 * Math.PI;

                // Set the rotation offset to the claw pickup position if enabled
        final boolean rotateAroundFront = RobotContainer.getRotateAroundFront();
        final Translation2d rotationOffset = rotateAroundFront ? new Translation2d() : Constants.RelativePositions.CLAW_PICKUP;

        drivetrain.drive(xSpeed, ySpeed, rot, false, rotationOffset);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false, new Translation2d());
    }
}