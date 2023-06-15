// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.arm;

import frc.robot.Constants;

public class ArmUtils {
    public static double armClicksToDegrees(double clicks) {
        return clicks*(360.0/(Constants.ArmConstants.gearRatio*4096));
    }
    public static double degreesToArmClicks(double degrees) {
        return degrees/(360/(Constants.ArmConstants.gearRatio*4096));
    }
}
