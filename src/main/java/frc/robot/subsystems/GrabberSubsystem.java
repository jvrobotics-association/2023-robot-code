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

    private boolean isWristInLowerRedZone = false;
    private boolean isWristInUpperRedZone = false;

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
        if (isWristInLowerRedZone) {
            if (speed < 0) {
                speed = 0;
            }
        } else if (isWristInUpperRedZone) {
            if (speed > 0) {
                speed = 0;
            }
        }
        // preventMovementInNoGoZones();
        wristMotor.set(speed);
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    public void setArmMotor(double speed) {
        if (getArmForwardLimitSwitch()) {
            if (speed > 0) {
                speed = 0;
            }
        } else if (getArmReverseLimitSwitch()) {
            if (speed < 0) {
                speed = 0;
            }
        }
        // preventMovementInNoGoZones();
        armMotor.set(-speed);
        SmartDashboard.putNumber("Arm Encoder", getArmEncoderPosition());
    }

    public double getArmEncoderPosition() {
        double position = armMotor.getEncoder().getPosition();
        SmartDashboard.putNumber("Arm Encoder", position);
        return position;
    }

    public void preventMovementInNoGoZones() {
        double armEncoderPosition = getArmEncoderPosition();
        double wristEncoderPosition = getWristPosition();

        boolean isWristInUpperGoZone = wristEncoderPosition > Constants.Claw.wristNoGoZoneUpperBound;
        boolean isWristInLowerGoZone = wristEncoderPosition < Constants.Claw.wristNoGoZoneLowerBound;

        if (armEncoderPosition < Constants.Arm.armNoGoZoneLowerBound) {
            if (isWristInLowerGoZone) {
                isWristInLowerRedZone = false;
                return;
            } else {
                isWristInLowerRedZone = true;
            }
            armMotor.set(0);
        } else if (armEncoderPosition > Constants.Arm.armNoGoZoneUpperBound && !isWristInUpperGoZone) {
            if (isWristInUpperGoZone) {
                isWristInUpperRedZone = false;
                return;
            } else {
                isWristInUpperRedZone = true;
            }
            armMotor.set(0);
        }

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
    }

    public boolean hasReachedTarget() {
        return Math.abs(getWristPosition() - wristEncoderTarget) <= Constants.Claw.wristMotorTolerance & hasArmReachedTarget();
    }

    
}
