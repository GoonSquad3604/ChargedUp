package frc.robot.util.arm;

import frc.robot.Constants;

public class ArmUtils {

    public static double armClicksToDegrees(double clicks) {
        return clicks*(360.0/(Constants.ArmConstants.gearRatio*4096));
    }

    public static double degreesToArmClicks(double degrees) {
        return degrees/(360.0/(Constants.ArmConstants.gearRatio*4096));
    }
    
}
