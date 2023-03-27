package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier isYLockedSup;
    private BooleanSupplier isSlowMode;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier isYLockedSup, BooleanSupplier isSlowMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.isYLockedSup = isYLockedSup;
        this.isSlowMode = isSlowMode;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (isSlowMode.getAsBoolean()) {
            translationVal = translationVal * 0.1;
            strafeVal = strafeVal * 0.1;
            rotationVal = rotationVal * 0.25;
        }

        /* Drive */
        // s_Swerve.drive(
        //     new Translation2d(isYLockedSup.getAsBoolean() ? 0 : translationVal, isYLockedSup.getAsBoolean() ? strafeVal * 1.0 : strafeVal).times(Constants.Swerve.maxSpeed), 
        //     rotationVal * Constants.Swerve.maxAngularVelocity, 
        //     !robotCentricSup.getAsBoolean(), 
        //     true
        // );

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}