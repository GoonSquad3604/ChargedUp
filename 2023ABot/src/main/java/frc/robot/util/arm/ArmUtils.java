package frc.robot.util.arm;

import frc.robot.Constants;

public class ArmUtils {
    /** Converts the clicks to degrees. */
    public static double armClicksToDegrees(double clicks) {
        return clicks*(360.0/(Constants.ArmConstants.gearRatio*4096));
    }
    /** Converts the degrees of the arm to clicks */
    public static double degreesToArmClicks(double degrees) {
        return degrees/(360.0/(Constants.ArmConstants.gearRatio*4096));
    }
    
}
