// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.auton;

import java.util.HashMap;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class AutonUtils {

    // public static PPSwerveControllerCommand getSwerveControllerCommand(PathPlannerTrajectory traj){
        
    //     return new PPSwerveControllerCommand(
    //         traj, 
    //         SwerveDrive.getInstance()::getPose,
    //         Constants.Swerve.swerveKinematics, 
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
    //         new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
    //         SwerveDrive.getInstance()::setModuleStates, 
    //         true,
    //         SwerveDrive.getInstance());
    // }



}
