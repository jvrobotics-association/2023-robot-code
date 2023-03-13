package frc.robot.commands.drive;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ZeroOdometry extends CommandBase {

    private Swerve swerve;

    public ZeroOdometry(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        swerve.zeroGyro();
    }
    
}
