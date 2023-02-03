package frc.robot.util.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

/*Returns each trajectory used in auton */
public final class Trajectories {
    
    public static PathPlannerTrajectory testTrajectory() {
        return PathPlanner.loadPath(
                "testpath",
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }
}
