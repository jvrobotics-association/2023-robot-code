package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {

    private double armEncoderTarget = 0;
    private DigitalInput armLimitSwitchForward = new DigitalInput(Constants.Arm.limitSwitchForwardId);
    private DigitalInput armLimitSwitchReverse = new DigitalInput(Constants.Arm.limitSwitchReverseId);

    private final CANSparkMax armMotor;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax wristMotor;
    private final DigitalInput wristLimitSwitchUp = new DigitalInput(Constants.Claw.wristLimitSwitchUpId);
    private final DigitalInput wristLimitSwitchDown = new DigitalInput(Constants.Claw.wristLimitSwitchDownId);

    private double wristEncoderTarget = 0;

    public GrabberSubsystem() {
        armMotor = new CANSparkMax(Constants.Arm.motorId, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Claw.intakeMotorId, CANSparkMax.MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.Claw.wristMotorId, CANSparkMax.MotorType.kBrushless);
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
        // If the wrist is agianst the up limit switch and is being told to move up, prevent it
        // from moving up any further to avoid damage to the robot.
        if (getWristLimitSwitchUp() && speed < 0) {
            wristMotor.set(0);
            return;
        }

        // If the wrist is agianst the down limit switch and is being told to move down, prevent it
        // from moving down any further to avoid damage to the robot.
        if (getWristLimitSwitchDown() && speed > 0) {
            wristMotor.set(0);
            return;
        }

        // Check if the current arm position is within the front red zone, if so it should check the position
        // of the wrist and then move the arm to a location to allow the wrist to move to the desired location.
        if (armMotor.getEncoder().getPosition() <= Constants.RedZoneValues.FORWARD_ARM_START.getPosition() && speed < 0) {
            wristMotor.set(0);
            return;
        }


        // Check if the current arm position is within the back red zone, if so it should check the position
        // of the wrist and then move the arm to a location to allow the wrist to move to the desired location.
        if (armMotor.getEncoder().getPosition() >= getDynamicWristRedZoneLimit() && speed > 0) {
            if (wristMotor.getEncoder().getPosition() >= getDynamicWristRedZoneLimit()) {
                wristMotor.set(0);
                return;
            }
        }

        // Move the wrist
        wristMotor.set(speed);
    }

    public double getDynamicWristRedZoneLimit() {
        // Calculate the lerp value based off the current arm position
        double lerp = (Constants.RedZoneValues.BACK_ARM_START.getPosition() - armMotor.getEncoder().getPosition()) / (Constants.RedZoneValues.BACK_ARM_START.getPosition() - 395.0);

        double decayValue = -Math.pow(lerp, 1.0/3.0) + 1;

        // Calculate the wrist limit based off the lerp value
        return -60.0 + (decayValue * (-25.0 - -60.0));
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    public void setArmMotor(double speed) {
        SmartDashboard.putNumber("Arm Encoder", getArmEncoderPosition());

        // If the arm is against the forward limit switch and it is being told to move forwards, prevent it
        // from moving forward any further to avoid damage to the robot.
        if (getArmForwardLimitSwitch() && speed > 0) {
            armMotor.set(0);
            armMotor.getEncoder().setPosition(0);
            return;
        }

        // If the arm is against the back limit switch and it is being told to move backwards, prevent it
        // from moving backwards any further to avoid damage to the robot.
        if (getArmReverseLimitSwitch() && speed < 0) {
            armMotor.set(0);
            return;
        }
    
        // Check if the current arm position is within the front red zone, if so
        // it should check the position of the wrist and move it if it is not in a proper position.
        if (armMotor.getEncoder().getPosition() <= Constants.RedZoneValues.FORWARD_ARM_START.getPosition() && speed > 0) {
            // Check if the wrist is beyond the limit, if so stop the arm and move the wrist back
            if (wristMotor.getEncoder().getPosition() <= Constants.RedZoneValues.FORWARD_WRIST_LIMIT.getPosition()) {
                // Stop the arm motor so that we can move the wrist
                armMotor.set(0);

                // Move the wrist until it is in the good zone
                setWristMotor(0.3);

                // Prevent the arm from moving until the wrist is in its proper location
                return;
            } else { 
                wristMotor.set(0);
            }
        }

        // Check if the current arm position is within the back red zone, if so
        // it should check the position of the wrist and move it if it is not in a proper position.
        if (armMotor.getEncoder().getPosition() >= Constants.RedZoneValues.BACK_ARM_START.getPosition() && speed < 0) {
            // Chcek if the wrist is beyong the limit, if so stop the arm and move the wrist back
            if (wristMotor.getEncoder().getPosition() >= Constants.RedZoneValues.BACK_WRIST_LIMIT.getPosition()) {
                // Stop the arm motor so that we can move the wrist
                armMotor.set(0);

                // Move the wrist until it is in the good zone
                setWristMotor(-0.3);

                // Prevent the arm from moving until the wrist is in its proper location
                return;
            } else {
                wristMotor.set(0);
            }
        }

        // Set the motor speed
        armMotor.set(-speed);
    }

    public double getArmEncoderPosition() {
        double position = armMotor.getEncoder().getPosition();
        SmartDashboard.putNumber("Arm Encoder", position);
        return position;
    }

    public boolean hasArmReachedTarget() {
        return (Math.abs(armEncoderTarget - getArmEncoderPosition()) <= Constants.Arm.allowedEncoderError);
    }

    public boolean getArmForwardLimitSwitch() {
        return !armLimitSwitchForward.get();
    }

    public boolean getArmReverseLimitSwitch() {
        return !armLimitSwitchReverse.get();
    }

    public void moveArmToTarget() {
        double delta = armEncoderTarget - getArmEncoderPosition();

        if (Math.abs(delta) > Constants.Arm.allowedEncoderError) {
            double direction = -(int) (delta / Math.abs(delta)) * Constants.Arm.maxSpeed;

            boolean hasStopped = false;

            if (getArmForwardLimitSwitch() && direction > 0) {
                hasStopped = true;
            } else if (getArmReverseLimitSwitch() && direction < 0) {
                hasStopped = true;
            }

            setArmMotor(hasStopped ? 0 : direction);
        } else {
            armMotor.stopMotor();
        }
    }

    public void setArmTargetEncoderValue(double target) {
        armEncoderTarget = target;
    }

    public void resetArmEncoder() {
        armMotor.getEncoder().setPosition(Constants.Arm.encoderZero);
    }

    // stops the motors
    public void stopMotor(CANSparkMax motor) {
        motor.set(0);
    }

    public boolean getWristLimitSwitchUp() {
        return !wristLimitSwitchUp.get();
    }

    public boolean getWristLimitSwitchDown() {
        return !wristLimitSwitchDown.get();
    }

    public boolean isArmMotorStoppedForward() {
        boolean isStopped = getArmForwardLimitSwitch();
        if (isStopped)
            armMotor.set(0);
        return isStopped;
    }

    public boolean isArmMotorStoppedBackwards() {
        boolean isStopped = getArmReverseLimitSwitch();
        if (isStopped)
            armMotor.set(0);
        return isStopped;
    }

    public boolean isWristMotorStopped() {
        SmartDashboard.putNumber("Wrist Motor Encoder", getWristPosition());

        boolean isStopped = getWristLimitSwitchUp() | getWristLimitSwitchDown();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public boolean isWristMotorStoppedUp() {
        SmartDashboard.putNumber("Wrist Motor Encoder", getWristPosition());
        boolean isStopped = getWristLimitSwitchUp();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public boolean isWristMotorStoppedDown() {
        SmartDashboard.putNumber("Wrist Motor Encoder", getWristPosition());
        boolean isStopped = getWristLimitSwitchDown();
        if (isStopped)
            wristMotor.set(0);
        return isStopped;
    }

    public void setWristEncoderTarget(double target) {
        wristEncoderTarget = target;
    }

    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }

    public void moveToTarget() {
        moveArmToTarget();
        moveWristToTarget();
    }

    public void moveWristToTarget() {
        double wristDelta = wristEncoderTarget - getWristPosition();

        if (Math.abs(wristDelta) > Constants.Arm.allowedEncoderError) {
            double direction = (int) (wristDelta / Math.abs(wristDelta)) * Constants.Claw.wristMotorSpeed;

            boolean hasStopped = false;

            if (getWristLimitSwitchDown() && direction > 0) {
                hasStopped = true;
            } else if (getWristLimitSwitchUp() && direction < 0) {
                hasStopped = true;
            }

            wristMotor.set(hasStopped ? 0 : direction);    
        } else {
            wristMotor.stopMotor();
        }
    }

    public void resetEncoder() {
        wristMotor.getEncoder().setPosition(0);
        armMotor.getEncoder().setPosition(0);
    }

    public boolean hasReachedTarget() {
        return Math.abs(getWristPosition() - wristEncoderTarget) <= Constants.Claw.wristMotorTolerance & hasArmReachedTarget();
    }
}
