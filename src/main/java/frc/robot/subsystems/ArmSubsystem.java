package frc.robot.subsystems;

import frc.robot.Constants;

public class ArmSubsystem {

    private Constants.ArmPositions currentPosition;

    // define the motors and encoders here for the primary, secondary, and wrist actions



    public ArmSubsystem() {
        // initialize the motors and encoders here
        currentPosition = Constants.ArmPositions.PARK;
    }

    public void setArmPosition(Constants.ArmPositions position) {
        currentPosition = position;
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
    
    
}
