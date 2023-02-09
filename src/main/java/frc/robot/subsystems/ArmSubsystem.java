package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private double primaryEncoderTarget = 0;
    private double secondaryEncoderTarget = 0;
    private Constants.ArmPositions currentPosition;

    // define the motors and encoders here for the primary, secondary, and wrist
    // actions
    private final CANSparkMax primaryMotor;
    private final CANSparkMax secondaryMotor;
    private final CANSparkMax wristMotor;

    public ArmSubsystem() {
        // initialize the motors and encoders here
        currentPosition = Constants.ArmPositions.PARK;
        primaryMotor = new CANSparkMax(Constants.Arm.PRIMARY_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        secondaryMotor = new CANSparkMax(Constants.Arm.SECONDARY_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.Arm.WRIST_MOTOR, CANSparkMax.MotorType.kBrushless);
    }

    // Sets value of the current arm position
    public void setArmPosition(Constants.ArmPositions position) {
        currentPosition = position;
    }

    /**
     * Checks if the primary motor is stopped. If so, it stops the motor and returns
     * true.
     * 
     * @return Whether the primary motor is stopped
     */
    public boolean isPrimaryMotorStopped() {
        double primaryCurrent = primaryMotor.getOutputCurrent();
        if (primaryCurrent > Constants.Arm.MAX_CURRENT) {
            stopMotor(primaryMotor);
            return true;
        }
        return false;
    }

    // controlls the motors
    public void setPrimaryMotor(double speed) {
        primaryMotor.set(speed);
    }

    public void setSecondaryMotor(double speed) {
        secondaryMotor.set(speed);
    }

    public void setWristMotor(double speed) {
        wristMotor.set(speed);
    }

    // TODO: Check to see if this is correct
    // gets the encoder positions
    public double getPrimaryEncoderPosition() {
        return primaryMotor.getEncoder().getPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getSecondaryEncoderPosition() {
        return secondaryMotor.getEncoder().getPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getWristEncoderPosition() {
        return wristMotor.getEncoder().getPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

    // Check to see if the arm is in the right position returns true if it is
    public boolean isInCurrentPosition() {
        // check if the arm is in the current position
        switch (currentPosition) {
            case BACK_POLE:
                // check if the arm is in the back pole position
                break;
            case FRONT_POLE:
                // check if the arm is in the front pole position
                break;
            case FLOOR_DROP:
                // check if the arm is in the floor drop position
                break;
            case FLOOR_PICKUP:
                // check if the arm is in the floor pickup position
                break;
            case SHELF_PICKUP:
                // check if the arm is in the slider pickup position
                break;
            case PARK:
                // check if the arm is in the park position
                break;
        }
        return false;
    }

    /*
     * Boolean on if the arm has reached its targeted position or not
     * 
     * @returns true if both joints have reached the target positions
     */
    public boolean hasReachedTarget() {
        System.out.println(primaryEncoderTarget - getPrimaryEncoderPosition());
        return (primaryEncoderTarget - getPrimaryEncoderPosition() <= Constants.Arm.ALLOWED_ENCODER_ERROR) && (secondaryEncoderTarget - getSecondaryEncoderPosition() <= Constants.Arm.ALLOWED_ENCODER_ERROR);
    }

    

    // stops the motors
    public void stopMotor(CANSparkMax motor) {
        motor.set(0);
    }

    /*
     * Moves the arm joints to the target encoder positions
     */
    public void moveToTarget() {
        // stores the distance that the primary and secondary joints need to travel
        double primaryDelta = primaryEncoderTarget - getPrimaryEncoderPosition();
        double secondaryDelta = secondaryEncoderTarget - getSecondaryEncoderPosition();

        // move the motors if not in allowed error
        if (primaryDelta > Constants.Arm.ALLOWED_ENCODER_ERROR) {
            int direction = (int) (primaryDelta / Math.abs(primaryDelta));
            primaryMotor.set(direction);
        } else {
            primaryMotor.stopMotor();
        }
        if (secondaryDelta > Constants.Arm.ALLOWED_ENCODER_ERROR) {
            int direction = (int) (secondaryDelta / Math.abs(secondaryDelta));
            secondaryMotor.set(direction);
        } else {
            secondaryMotor.stopMotor();
        }
    }

    public void setTargetEncoderValues(Translation2d position) {
        // calculate the target positions for the motors
        secondaryEncoderTarget = convertThetaToEncoder(secondaryThetaFromPosition(position), 0, Constants.Arm.SECONDARY_ARM_GEAR_RATIO);
        primaryEncoderTarget = convertThetaToEncoder(primaryThetaFromPosition(position, secondaryThetaFromPosition(position)), 0, Constants.Arm.PRIMARY_ARM_GEAR_RATIO);
    }

    /*
     * Moves the arms to predefined positions
     */
    // TODO: Finish this method
    public void moveToCurrentPosition() {
        // move the arm to the current position
        switch (currentPosition) {
            case BACK_POLE:
                // move the arm to the back pole position
                break;
            case FRONT_POLE:
                // move the arm to the front pole position
                break;
            case FLOOR_DROP:
                // move the arm to the floor drop position
                break;
            case FLOOR_PICKUP:
                // move the arm to the floor pickup position
                break;
            case SHELF_PICKUP:
                // move the arm to the slider pickup position
                break;
            case PARK:
                // move the arm to the park position
                break;
        }
    }

    /*
     * Resets the encoders to zero
     */
    public void resetEncoders() {
        primaryMotor.getEncoder().setPosition(0);
    }

    /*
     * Checks to see if the secondary motor is stopped by checking the current.
     * 
     * @return Whether the secondary motor is stopped
     */
    public boolean isSecondaryMotorStopped() {
        double secondaryCurrent = secondaryMotor.getOutputCurrent();
        if (secondaryCurrent > Constants.Arm.MAX_CURRENT) {
            stopMotor(secondaryMotor);
            return true;
        }
        return false;
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
    public double convertEncoderToTheta(double encoderValue, double zeroPoint, double gearRatio) {
        return (encoderValue - zeroPoint) * 2 * Math.PI / (gearRatio * Constants.Arm.ENCODER_TICKS_PER_REVOLUTION);
    }

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
    public double convertThetaToEncoder(double theta, double zeroPoint, double gearRatio) {
        return (theta * gearRatio) / (2 * Math.PI) + zeroPoint;
    }

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
    public double secondaryThetaFromPosition(Translation2d position) {
        // store the x and y values of the position
        double x = position.getX();
        double y = position.getY();
        // calculate the secondary theta
        double theta = -Math.acos((x * x + y * y - Constants.Arm.PRIMARY_ARM_LENGTH * Constants.Arm.PRIMARY_ARM_LENGTH
                - Constants.Arm.SECONDARY_ARM_LENGTH * Constants.Arm.SECONDARY_ARM_LENGTH)
                / (2 * Constants.Arm.PRIMARY_ARM_LENGTH * Constants.Arm.SECONDARY_ARM_LENGTH));
        return theta;
    }

    /*
     * Calculates the primary theta from the target position
     * 
     * @param position The target position
     * 
     * @param secondaryTheta The secondary theta
     * 
     * @return The primary theta
     */
    public double primaryThetaFromPosition(Translation2d position, double secondaryTheta) {
        // store the x and y values of the position
        double x = position.getX();
        double y = position.getY();
        // calculate the primary theta
        double theta = Math.atan2(y, x) + Math.atan2(Constants.Arm.SECONDARY_ARM_LENGTH * Math.sin(secondaryTheta),
                Constants.Arm.PRIMARY_ARM_LENGTH + Constants.Arm.SECONDARY_ARM_LENGTH * Math.cos(secondaryTheta));
        return theta;
    }

}
