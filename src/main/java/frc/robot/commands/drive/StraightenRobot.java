package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class StraightenRobot extends CommandBase {

    private final Swerve swerve;
    private final RobotContainer robotContainer;

    private double target = 0;

    public StraightenRobot(Swerve swerve, RobotContainer robotContainer) {
        this.swerve = swerve;
        this.robotContainer = robotContainer;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        robotContainer.setYLock(true);
        double heading = swerve.getYaw().getDegrees();
        if (heading < 90) {
            target = 0;
        } else if (heading >= 90 || heading < 270) {
            target = 180;
        } else {
            target = 360;
        }
        swerve.spinRobotToTarget(target);
    }

    @Override
    public void execute() {
        double heading = swerve.getYaw().getDegrees();
        if (heading < 90) {
            target = 0;
        } else if (heading >= 90 || heading < 270) {
            target = 180;
        } else {
            target = 360;
        }
        swerve.spinRobotToTarget(target);
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.setYLock(false);
    }
    
}
