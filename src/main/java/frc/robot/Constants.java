package frc.robot;


/*
 * This class is where the constants for the robot are stored. This is the sources of all modifyable values to tune the robot.
 */
public class Constants {

    public static final class Controllers {
        public static final int DRIVER_CONTROLS_PORT = 0;
        public static final int CONTROL_PANEL_PORT = 1;
    }

    public static enum ArmPositions {
        BACK_POLE, FRONT_POLE, FLOOR_DROP, FLOOR_PICKUP, SLIDER_PICKUP, PARK
    }
}
