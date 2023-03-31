package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class StraightenRobot extends CommandBase {

    private final Swerve swerve;
    private final RobotContainer robotContainer;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier translationSupplier;

    private double target = 0;

    public StraightenRobot(Swerve swerve, RobotContainer robotContainer, DoubleSupplier strafeSup, DoubleSupplier translationSupplier) {
        this.swerve = swerve;
        this.robotContainer = robotContainer;
        this.strafeSup = strafeSup;
        this.translationSupplier = translationSupplier;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        target = 0;
        swerve.spinRobotToTarget(target, strafeSup.getAsDouble() * 0.75, translationSupplier.getAsDouble() * 0.75);
    }

    @Override
    public void execute() {
        target = 0;
        swerve.spinRobotToTarget(target, strafeSup.getAsDouble() * 0.75, translationSupplier.getAsDouble() * 0.75);
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.setYLock(false);
    }
    
}
