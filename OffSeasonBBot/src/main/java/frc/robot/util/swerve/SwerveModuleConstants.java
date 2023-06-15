package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * This is used for keeping track of constants for a single swerve module.
 * 
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID; 
        this.angleOffset = angleOffset;
    }



}
