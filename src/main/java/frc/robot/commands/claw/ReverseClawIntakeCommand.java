package frc.robot.commands.claw;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class ReverseClawIntakeCommand extends CommandBase {

    // Required subsystems
    private final IntakeSubsystem intakeSubsystem;
    private final Swerve swerve;

    public ReverseClawIntakeCommand(IntakeSubsystem _intakeSubsystem, Swerve _swerve){
        this.intakeSubsystem = _intakeSubsystem;
        this.swerve = _swerve;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeMotor(0.25);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(-0.05, 0), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
        swerve.drive(new Translation2d(), 0, false, true);
    }
    
}
