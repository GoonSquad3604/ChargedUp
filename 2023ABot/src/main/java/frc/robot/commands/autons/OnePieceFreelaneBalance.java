// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import frc.robot.commands.drive.Stop;
import frc.robot.util.auton.AutonUtils;
import frc.robot.util.auton.GoonAutonCommand;
import frc.robot.util.auton.Trajectories;

/** Add your docs here. */
public class OnePieceFreelaneBalance extends GoonAutonCommand{

  public OnePieceFreelaneBalance(){
    super.addCommands(
      AutonUtils.getSwerveControllerCommand(Trajectories.twoConeFreelane())
  );
    super.setInitialPose(Trajectories.twoConeFreelane());
    new Stop();
  }
}

