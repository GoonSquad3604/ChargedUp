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

    public static PathPlannerTrajectory freeLaneMeterBack() {
        return PathPlanner.loadPath(
                "FreeLaneMeterBack",
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }
    public static PathPlannerTrajectory midMeterBack() {
        return PathPlanner.loadPath(
                "MidMeterBack",
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    public static PathPlannerTrajectory offLaneMeterBack() {
        return PathPlanner.loadPath(
                "OuterLaneMeterBack",
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }
    
    public static PathPlannerTrajectory twoPieceFreeLane() {
        return PathPlanner.loadPath(
                "TwoPieceFreelane",
                Constants.AutoConstants.kSlowMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    public static PathPlannerTrajectory offLaneBackUp() {
        return PathPlanner.loadPath(
                "OffLaneBackUp",
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    public static PathPlannerTrajectory ontoPlatform() {
        return PathPlanner.loadPath(
                "OntoPlatform",
                Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
    }

    public static PathPlannerTrajectory ontoPlatformFromCube() {
        return PathPlanner.loadPath(
                "OntoPlatformFromCubeScore",
                Constants.AutoConstants.kSlowerMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kSlowerMaxAccelerationMetersPerSecondSquared);
    }
    
}
