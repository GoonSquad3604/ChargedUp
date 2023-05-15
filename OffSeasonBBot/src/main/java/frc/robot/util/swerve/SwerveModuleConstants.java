package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
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
