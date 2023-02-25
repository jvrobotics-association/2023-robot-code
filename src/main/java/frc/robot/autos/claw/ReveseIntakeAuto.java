package frc.robot.autos.claw;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ReveseIntakeAuto extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private double startTime;
    private final double duration = 1.0;

    public ReveseIntakeAuto(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        clawSubsystem.setIntakeMotor(-1.0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > duration;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setIntakeMotor(0.0);
    }
    
}
