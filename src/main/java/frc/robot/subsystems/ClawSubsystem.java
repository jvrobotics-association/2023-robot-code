package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {

    // The motor to control the intake and the solenoid to control the claw
    private final CANSparkMax intakeMotor;
    private final CANSparkMax wristMotor;
    private final DigitalInput wristLimitSwitchUp;
    private final DigitalInput wristLimitSwitchDown;

    private double wristEncoderTarget = 0;

    public ClawSubsystem() {
        // Initialize the motors and solenoids
        intakeMotor = new CANSparkMax(Constants.Claw.intakeMotorId, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(50);
        wristMotor = new CANSparkMax(Constants.Claw.wristMotorId, MotorType.kBrushless);
        wristLimitSwitchUp = new DigitalInput(Constants.Claw.wristLimitSwitchUpId);
        wristLimitSwitchDown = new DigitalInput(Constants.Claw.wristLimitSwitchDownId);
    }

    // Sets the speed of the intake motor
    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    // Sets the speed of the wrist motor
    public void setWristMotor(double speed) {
        wristMotor.set(speed);
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    public boolean getWristLimitSwitchUp() {
        return !wristLimitSwitchUp.get();
    }

    public boolean getWristLimitSwitchDown() {
        return !wristLimitSwitchDown.get();
    }

    public boolean isWristMotorStopped() {
        boolean isStopped = getWristLimitSwitchUp() | getWristLimitSwitchDown();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public boolean isWristMotorStoppedUp() {
        boolean isStopped = getWristLimitSwitchUp();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public boolean isWristMotorStoppedDown() {
        boolean isStopped = getWristLimitSwitchDown();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public void setWristEncoderTarget(double target) {
        wristEncoderTarget = target;
    }

    public double getWristEncoder() {
        return wristMotor.getEncoder().getPosition();
    }

    public void moveToTarget() {
        if (getWristEncoder() < wristEncoderTarget) {
            setWristMotor(Constants.Claw.wristMotorSpeed);
        } else if (getWristEncoder() > wristEncoderTarget) {
            setWristMotor(-Constants.Claw.wristMotorSpeed);
        } else {
            stopWristMotor();
        }
    }

    public void resetEncoder() {
        wristMotor.getEncoder().setPosition(0);
    }

    public boolean hasReachedTarget() {

        isWristMotorStopped();

        return Math.abs(getWristEncoder() - wristEncoderTarget) < Constants.Claw.wristMotorTolerance;
    }

}