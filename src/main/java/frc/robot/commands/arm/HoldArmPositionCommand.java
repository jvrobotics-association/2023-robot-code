package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class HoldArmPositionCommand extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    public HoldArmPositionCommand(GrabberSubsystem _grabberSubsystem) {
        this.grabberSubsystem = _grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setArmTargetEncoderValue(grabberSubsystem.getArmEncoderPosition());
        grabberSubsystem.setWristEncoderTarget(grabberSubsystem.getWristPosition());
    }

    @Override
    public void execute() {
        // grabberSubsystem.moveToTarget();
        grabberSubsystem.hasReachedTarget();
        grabberSubsystem.printEncoders();
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setArmMotor(0);
        grabberSubsystem.setWristMotor(0);
    }
    
}
