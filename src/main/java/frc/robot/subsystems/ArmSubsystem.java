package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private Constants.ArmPositions currentPosition;

    // define the motors and encoders here for the primary, secondary, and wrist
    // actions
    private final TalonSRX primaryMotor;
    private final TalonSRX secondaryMotor;
    private final TalonSRX wristMotor;

    public ArmSubsystem() {
        // initialize the motors and encoders here
        currentPosition = Constants.ArmPositions.PARK;
        primaryMotor = new TalonSRX(Constants.Arm.PRIMARY_MOTOR);
        secondaryMotor = new TalonSRX(Constants.Arm.SECONDARY_MOTOR);
        wristMotor = new TalonSRX(Constants.Arm.WRIST_MOTOR);

        TalonSRXConfiguration primaryConfig = new TalonSRXConfiguration();
        primaryConfig.continuousCurrentLimit = Constants.Arm.MAX_CURRENT;

        primaryMotor.configAllSettings(primaryConfig);
        secondaryMotor.configAllSettings(primaryConfig);
        wristMotor.configAllSettings(primaryConfig);
    }

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
        double primaryCurrent = primaryMotor.getStatorCurrent();
        if (primaryCurrent > Constants.Arm.MAX_CURRENT) {
            stopMotor(primaryMotor);
            return true;
        }
        return false;
    }

    public void setPrimaryMotor(double speed) {
        primaryMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setSecondaryMotor(double speed) {
        secondaryMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setWristMotor(double speed) {
        wristMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public double getPrimaryEncoderPosition() {
        return primaryMotor.getSelectedSensorPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getSecondaryEncoderPosition() {
        return secondaryMotor.getSelectedSensorPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

    public double getWristEncoderPosition() {
        return wristMotor.getSelectedSensorPosition() % Constants.Arm.ENCODER_TICKS_PER_REVOLUTION;
    }

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

    public void stopMotor(TalonSRX motor) {
        motor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void moveTo(Translation2d position) {      
        double targetSecondarytheta = secondaryThetaFromPosition(position);
        double targetPrimarytheta = primaryThetaFromPosition(position, targetSecondarytheta);

        primaryMotor.set(TalonSRXControlMode.Position, convertThetaToEncoder(targetPrimarytheta, 0, Constants.Arm.PRIMARY_ARM_GEAR_RATIO));
        secondaryMotor.set(TalonSRXControlMode.Position, convertThetaToEncoder(targetSecondarytheta, 0, Constants.Arm.SECONDARY_ARM_GEAR_RATIO));
    }

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

    public void resetEncoders() {
        primaryMotor.setSelectedSensorPosition(0);
    }

    public boolean isSecondaryMotorStopped() {
        double secondaryCurrent = secondaryMotor.getStatorCurrent();
        if (secondaryCurrent > Constants.Arm.MAX_CURRENT) {
            stopMotor(secondaryMotor);
            return true;
        }
        return false;
    }

    public double convertEncoderToTheta(double encoderValue, double zeroPoint, double gearRatio) {
        return (encoderValue - zeroPoint) * 2 * Math.PI / (gearRatio * Constants.Arm.ENCODER_TICKS_PER_REVOLUTION);
    }

    public double convertThetaToEncoder(double theta, double zeroPoint, double gearRatio) {
        return (theta * gearRatio) / (2 * Math.PI) + zeroPoint;
    }

    // Work in progress
    // public double calculateDeltaTheta(double speed, double gearRatio) {
    //     return speed / gearRatio;
    // }

    public double secondaryThetaFromPosition(Translation2d position) {
        double x = position.getX();
        double y = position.getY();
        double theta = -Math.acos((x * x + y * y - Constants.Arm.PRIMARY_ARM_LENGTH * Constants.Arm.PRIMARY_ARM_LENGTH
                - Constants.Arm.SECONDARY_ARM_LENGTH * Constants.Arm.SECONDARY_ARM_LENGTH)
                / (2 * Constants.Arm.PRIMARY_ARM_LENGTH * Constants.Arm.SECONDARY_ARM_LENGTH));
        return theta;
    }

    public double primaryThetaFromPosition(Translation2d position, double secondaryTheta) {
        double x = position.getX();
        double y = position.getY();
        double theta = Math.atan2(y, x) + Math.atan2(Constants.Arm.SECONDARY_ARM_LENGTH * Math.sin(secondaryTheta),
                Constants.Arm.PRIMARY_ARM_LENGTH + Constants.Arm.SECONDARY_ARM_LENGTH * Math.cos(secondaryTheta));
        return theta;
    }

}
