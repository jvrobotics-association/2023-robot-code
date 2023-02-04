package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private Constants.ArmPositions currentPosition;

    // define the motors and encoders here for the primary, secondary, and wrist
    // actions
    private final TalonSRX primaryMotor;
    private final TalonSRX secondaryMotor;


    public ArmSubsystem() {
        // initialize the motors and encoders here
        currentPosition = Constants.ArmPositions.PARK;
        primaryMotor = new TalonSRX(Constants.Arm.PRIMARY_MOTOR);
        secondaryMotor = new TalonSRX(Constants.Arm.SECONDARY_MOTOR);

        TalonSRXConfiguration primaryConfig = new TalonSRXConfiguration();
        primaryConfig.continuousCurrentLimit = Constants.Arm.MAX_CURRENT;

        primaryMotor.configAllSettings(primaryConfig);
        secondaryMotor.configAllSettings(primaryConfig);
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

}
