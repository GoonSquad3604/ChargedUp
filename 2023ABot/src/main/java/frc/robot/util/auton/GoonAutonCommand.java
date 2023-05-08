// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoonAutonCommand extends SequentialCommandGroup {
  /** Creates a new GoonAutonCommand. */

  private Pose2d initialPose;


  public GoonAutonCommand() {
    // Add your commands in the addCommands() call, e.g.
    
  }

  protected void setInitialPose(PathPlannerTrajectory initialTrajectory) {
    this.initialPose = new Pose2d(initialTrajectory.getInitialPose().getTranslation(),
            initialTrajectory.getInitialState().holonomicRotation);
  }

  public Pose2d getInitialPose() {
      return initialPose;
  }
}
