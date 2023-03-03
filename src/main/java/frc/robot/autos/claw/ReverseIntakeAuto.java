package frc.robot.autos.claw;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ReverseIntakeAuto extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private double startTime;

    public ReverseIntakeAuto(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        clawSubsystem.setIntakeMotor(0.5);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopIntakeMotor();
    }
    
}
