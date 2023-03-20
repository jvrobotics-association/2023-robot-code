package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // private double primaryEncoderTarget = 0;
    // private double secondaryEncoderTarget = 0;

    // private DigitalInput primaryLimitSwitchForward = new DigitalInput(Constants.Arm.primaryLimitSwitchForwardId);
    // private DigitalInput primaryLimitSwitchReverse = new DigitalInput(Constants.Arm.primaryLimitSwitchReverseId);
    // private DigitalInput secondaryLimitSwitchUp = new DigitalInput(Constants.Arm.secondaryLimitSwitchUpId);
    // private DigitalInput secondaryLimitSwitchDown = new DigitalInput(Constants.Arm.secondaryLimitSwitchDownId);

    private double encoderTarget = 0;
    private DigitalInput limitSwitchForward = new DigitalInput(Constants.Arm.limitSwitchForwardId);
    private DigitalInput limitSwitchReverse = new DigitalInput(Constants.Arm.limitSwitchReverseId);

    private final CANSparkMax motor;


    // define the motors and encoders here for the primary, secondary, and wrist
    // actions
    // private final CANSparkMax primaryMotor;
    // private final CANSparkMax secondaryMotor;

    public ArmSubsystem() {
        // initialize the motors and encoders here
        // primaryMotor = new CANSparkMax(Constants.Arm.primaryArmMotorId, CANSparkMax.MotorType.kBrushless);
        // secondaryMotor = new CANSparkMax(Constants.Arm.secondaryArmMotorId, CANSparkMax.MotorType.kBrushless);

        motor = new CANSparkMax(Constants.Arm.motorId, CANSparkMax.MotorType.kBrushless);
    }

    /**
     * Checks if the primary motor is stopped. If so, it stops the motor and returns
     * true.
     * 
     * @return Whether the primary motor is stopped
     */
    // public boolean isPrimaryMotorStoppedForward() {
    //     boolean isStopped = getPrimaryForwardLimitSwitch();
    //     if (isStopped)
    //         primaryMotor.set(0);
    //     return isStopped;
    // }

    // public boolean isPrimaryMotorStoppedBackwards() {
    //     boolean isStopped = getPrimaryReverseLimitSwitch();
    //     if (isStopped)
    //         primaryMotor.set(0);
    //     return isStopped;
    // }

    /**
     * Checks if the secondary motor is stopped. If so, it stops the motor and returns
     * true.
     * 
     * @return Whether the secondary motor is stopped
     */
    // public boolean isSecondaryMotorStoppedDown() {
    //     boolean isStopped = getSecondaryLimitSwitchDown();
    //     if (isStopped)
    //         secondaryMotor.set(0);
    //     return isStopped;
    // }

    // public boolean isSecondaryMotorStoppedUp() {
    //     boolean isStopped = getSecondaryLimitSwitchUp();
    //     if (isStopped)
    //         secondaryMotor.set(0);
    //     return isStopped;
    // }

    public boolean isMotorStoppedForward() {
        boolean isStopped = getForwardLimitSwitch();
        if (isStopped)
            motor.set(0);
        return isStopped;
    }

    public boolean isMotorStoppedBackwards() {
        boolean isStopped = getReverseLimitSwitch();
        if (isStopped)
            motor.set(0);
        return isStopped;
    }



    // controlls the motors and makes sure they are not going past the limit switches
    // public void setPrimaryMotor(double speed) {
    //     if (getPrimaryForwardLimitSwitch()) {
    //         if (speed > 0) {
    //             speed = 0;
    //         }
    //     } else if (getPrimaryReverseLimitSwitch()) {
    //         if (speed < 0) {
    //             speed = 0;
    //         }
    //     }
    //     primaryMotor.set(speed);
    //     SmartDashboard.putNumber("Primary Arm Encoder", getPrimaryEncoderPosition());
    // }

    // public void setSecondaryMotor(double speed) {
    //     if (getSecondaryLimitSwitchDown()) {
    //         if (speed < 0) {
    //             speed = 0;
    //         }
    //     }
    //     if (getSecondaryLimitSwitchUp()) {
    //         if (speed > 0) {
    //             speed = 0;
    //         }
    //     }
    //     speed = -speed;
    //     secondaryMotor.set(speed);
    //     SmartDashboard.putNumber("Secondary Arm Encoder", getSecondaryEncoderPosition());
    // }

    public void setMotor(double speed) {
        if (getForwardLimitSwitch()) {
            if (speed > 0) {
                speed = 0;
            }
        } else if (getReverseLimitSwitch()) {
            if (speed < 0) {
                speed = 0;
            }
        }
        motor.set(speed);
        SmartDashboard.putNumber("Arm Encoder", getEncoderPosition());
    }

    // // gets the encoder positions
    // public double getPrimaryEncoderPosition() {
    //     double position = primaryMotor.getEncoder().getPosition();
    //     SmartDashboard.putNumber("Primary Arm Encoder", position);
    //     return position;
    // }

    // public double getSecondaryEncoderPosition() {
    //     double position = secondaryMotor.getEncoder().getPosition();
    //     SmartDashboard.putNumber("Secondary Arm Encoder", position);
    //     return position;
    // }

    public double getEncoderPosition() {
        double position = motor.getEncoder().getPosition();
        SmartDashboard.putNumber("Arm Encoder", position);
        return position;
    }

    
    /*
     * Boolean on if the arm has reached its targeted position or not
     * 
     * @return true if both joints have reached the target positions
     */
    public boolean hasReachedTarget() {
        // SmartDashboard.putNumber("Primary Arm Distance to Target", primaryEncoderTarget-getPrimaryEncoderPosition());
        // SmartDashboard.putNumber("Secondary Arm Distance to Target", secondaryEncoderTarget-getSecondaryEncoderPosition());
        // check to make sure that the arms are not past the limit switches
        // isPrimaryMotorStoppedBackwards();
        // isPrimaryMotorStoppedForward();
        // isSecondaryMotorStoppedDown();
        // isSecondaryMotorStoppedUp();

        // return (Math.abs(primaryEncoderTarget - getPrimaryEncoderPosition()) <= Constants.Arm.allowedEncoderError) & (Math.abs(secondaryEncoderTarget - getSecondaryEncoderPosition()) <= Constants.Arm.allowedEncoderError);
        return (Math.abs(encoderTarget - getEncoderPosition()) <= Constants.Arm.allowedEncoderError);
    }
    

    // public boolean hasReachedPrimaryTarget() {
    //     return (Math.abs(primaryEncoderTarget - getPrimaryEncoderPosition()) <= Constants.Arm.allowedEncoderError);
    // }

    // public boolean isPrimaryOffZeroArea() {
    //     return (getPrimaryEncoderPosition() > Constants.Arm.zeroAreaPrimaryEncoderValue);
        
    // }

    // public boolean hasReachedSecondaryTarget() {
    //     return (Math.abs(secondaryEncoderTarget - getSecondaryEncoderPosition()) <= Constants.Arm.allowedEncoderError);
    // }

    // public boolean isSecondaryOffZeroArea() {
    //     return (getSecondaryEncoderPosition() < Constants.Arm.zeroAreaSecondaryEncoderValue);
    // }

    // public boolean isOffZeroArea() {
    //     return (getEncoderPosition() > Constants.Arm.zeroAreaEncoderValue);
    // }

    // public boolean getPrimaryForwardLimitSwitch() {
    //     return !primaryLimitSwitchForward.get();
    // }

    // public boolean getPrimaryReverseLimitSwitch() {
    //     return !primaryLimitSwitchReverse.get();
    // }

    // public boolean getSecondaryLimitSwitchUp() {
    //     return !secondaryLimitSwitchUp.get();
    // }

    // public boolean getSecondaryLimitSwitchDown() {
    //     return !secondaryLimitSwitchDown.get();
    // }

    public boolean getForwardLimitSwitch() {
        return !limitSwitchForward.get();
    }

    public boolean getReverseLimitSwitch() {
        return !limitSwitchReverse.get();
    }

    

    // stops the motors
    public void stopMotor(CANSparkMax motor) {
        motor.set(0);
    }

    /*
     * Moves the arm joints to the target encoder positions
    //  */
    // public void movePrimaryToTarget() {
    //     // stores the distance that the primary and secondary joints need to travel
    //     double primaryDelta = primaryEncoderTarget - getPrimaryEncoderPosition();
        

    //     // move the motors if not in allowed error
    //     if (Math.abs(primaryDelta) > Constants.Arm.allowedEncoderError) {
    //         double direction = (int) (primaryDelta / Math.abs(primaryDelta)) * Constants.Arm.primaryArmMaxSpeed;

    //         boolean hasStopped = false;

    //         if (getPrimaryForwardLimitSwitch() && direction > 0) {
    //             hasStopped = true;
    //         } else if (getPrimaryReverseLimitSwitch() && direction < 0) {
    //             hasStopped = true;
    //         }        

    //         setPrimaryMotor(hasStopped ? 0 : direction);
    //     } else {
    //         primaryMotor.stopMotor();
    //     }
    
    // }



    // public void moveSecondaryToTarget() {

    //     double secondaryDelta = secondaryEncoderTarget - getSecondaryEncoderPosition();

    //     if (Math.abs(secondaryDelta) > Constants.Arm.allowedEncoderError) {
    //         double direction = -(int) (secondaryDelta / Math.abs(secondaryDelta)) * Constants.Arm.secondaryArmMaxSpeed;

    //         boolean hasStopped = false;

    //         if (getSecondaryLimitSwitchDown() && direction < 0) {
    //             hasStopped = true;
    //         } else if (getSecondaryLimitSwitchUp() && direction > 0) {
    //             hasStopped = true;
    //         }

    //         setSecondaryMotor(hasStopped ? 0 : direction);
    //     } else {
    //         secondaryMotor.stopMotor();
    //     }
    // }

    public void moveToTarget() {
        double delta = encoderTarget - getEncoderPosition();

        if (Math.abs(delta) > Constants.Arm.allowedEncoderError) {
            double direction = (int) (delta / Math.abs(delta)) * Constants.Arm.maxSpeed;

            boolean hasStopped = false;

            if (getForwardLimitSwitch() && direction > 0) {
                hasStopped = true;
            } else if (getReverseLimitSwitch() && direction < 0) {
                hasStopped = true;
            }

            setMotor(hasStopped ? 0 : direction);
        } else {
            motor.stopMotor();
        }
    }

    // public void setTargetEncoderValues(Translation2d position) {
    //     // calculate the target positions for the motors
    //     secondaryEncoderTarget = convertThetaToEncoder(secondaryThetaFromPosition(position), 0, Constants.Arm.secondaryArmGearRatio);
    //     primaryEncoderTarget = convertThetaToEncoder(primaryThetaFromPosition(position, secondaryThetaFromPosition(position)), 0, Constants.Arm.primaryArmGearRatio);
    // }

    // public void setTargetEncoderValues(double primaryTarget, double secondaryTarget) {
    //     primaryEncoderTarget = primaryTarget;
    //     secondaryEncoderTarget = secondaryTarget;
    // }

    public void setTargetEncoderValue(double target) {
        encoderTarget = target;
    }
    

    /*
     * Resets the encoders to zero
     */
    // public void resetEncoders() {
    //     primaryMotor.getEncoder().setPosition(Constants.Arm.primaryArmEncoderZero);
    //     secondaryMotor.getEncoder().setPosition(Constants.Arm.secondaryArmEncoderZero);
    // }

    public void resetEncoder() {
        motor.getEncoder().setPosition(Constants.Arm.encoderZero);
    }

    /*
     * Converts the encoder value to the angle of the arm
     * 
     * @param encoderValue The encoder value to convert
     * 
     * @param zeroPoint The zero point of the arm (if it is not at zero else pass in
     * 0)
     * 
     * @param gearRatio The gear ratio of the arm
     * 
     * @return The angle of the arm
     */
    // public double convertEncoderToTheta(double encoderValue, double zeroPoint, double gearRatio) {
    //     return (encoderValue - zeroPoint) * 2 * Math.PI / (gearRatio * Constants.Arm.encoderTicksPerRevolution);
    // }

    /*
     * Converts the angle of the arm to the encoder value
     * 
     * @param theta The angle of the arm
     * 
     * @param zeroPoint The zero point of the arm (if it is not at zero else pass in
     * 0)
     * 
     * @param gearRatio The gear ratio of the arm
     * 
     * @return The encoder value
     */
    // public double convertThetaToEncoder(double theta, double zeroPoint, double gearRatio) {
    //     return (theta * gearRatio) / (2 * Math.PI) + zeroPoint;
    // }

    // Work in progress
    // public double calculateDeltaTheta(double speed, double gearRatio) {
    // return speed / gearRatio;
    // }

    /*
     * Calculates the secondary theta from the target position
     * 
     * @param position The target position
     * 
     * @return The secondary theta
     */
    // public double secondaryThetaFromPosition(Translation2d position) {
    //     // store the x and y values of the position
    //     double x = position.getX();
    //     double y = position.getY();
    //     // calculate the secondary theta
    //     double theta = -Math.acos((x * x + y * y - Constants.Arm.primaryArmLength * Constants.Arm.primaryArmLength
    //             - Constants.Arm.secondaryArmLength * Constants.Arm.secondaryArmLength)
    //             / (2 * Constants.Arm.primaryArmLength * Constants.Arm.secondaryArmLength));
    //     return theta;
    // }

    /*
     * Calculates the primary theta from the target position
     * 
     * @param position The target position
     * 
     * @param secondaryTheta The secondary theta
     * 
     * @return The primary theta
     */
    // public double primaryThetaFromPosition(Translation2d position, double secondaryTheta) {
    //     // store the x and y values of the position
    //     double x = position.getX();
    //     double y = position.getY();
    //     // calculate the primary theta
    //     double theta = Math.atan2(y, x) + Math.atan2(Constants.Arm.secondaryArmLength * Math.sin(secondaryTheta),
    //             Constants.Arm.primaryArmLength + Constants.Arm.secondaryArmLength * Math.cos(secondaryTheta));
    //     return theta;
    // }

}
