package frc.robot.autos.claw;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class ReverseIntakeAuto extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final Swerve swerve;
    private double startTime;

    public ReverseIntakeAuto(IntakeSubsystem _intakeSubsystem, Swerve _swerve) {
        this.intakeSubsystem = _intakeSubsystem;
        this.swerve = _swerve;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeMotor(0.5);
        swerve.drive(new Translation2d(-0.05, 0), 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
        swerve.drive(new Translation2d(), 0, false, true);
    }
    
}
