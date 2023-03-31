package frc.robot.autos.claw;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeFastAuto extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem;
    private double timeStamp;
    private double duration;

    public ReverseIntakeFastAuto(IntakeSubsystem _intakeSubsystem, double _duration){
        this.intakeSubsystem = _intakeSubsystem;
        this.duration = _duration;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeMotor(1.00);
        timeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timeStamp > duration;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
    }
}
