
package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SwerveUtils {

    public static class SwerveOptimizer{
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInApprpriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90){
                targetSpeed = -targetSpeed;
                target
            }
        }
    }

    public static class Conversions{

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0/2048);
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

    
        public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
            return positionCounts * (circumference / (gearRatio * 2048.0));
        }

       
        public static double MetersToFalcon(double meters, double circumference, double gearRatio){
            return meters / (circumference / (gearRatio * 2048.0));
        }

      
        public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }

       
        public static double falconToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 2048.0));
        }

       
        public static double degreesToFalcon(double degrees, double gearRatio) {
            return degrees / (360.0 / (gearRatio * 2048.0));
        }

        


    }


}
