package frc.robot.autos.claw;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class ReverseIntakeAuto extends CommandBase {

    private final GrabberSubsystem grabberSubsystem;
    private double startTime;

    public ReverseIntakeAuto(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        grabberSubsystem.setIntakeMotor(0.5);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.stopIntakeMotor();
    }
    
}
