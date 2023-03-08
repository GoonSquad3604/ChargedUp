// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.utils.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.util.auton.AutonUtils;
import frc.robot.util.auton.GoonAutonCommand;
import frc.robot.util.auton.Trajectories;

/** Add your docs here. */
public class TwoPieceFreelane extends GoonAutonCommand{

    Arm m_Arm;

  public TwoPieceFreelane(){
    super.addCommands(
        new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCone)),
        new ArmHigh(Constants.ArmConstants.highConeShoulder, Constants.ArmConstants.highConeElbow),
        new Wait(1),
        new InstantCommand(() -> m_Arm.clawTo(0)),
        new Wait(1),

      AutonUtils.getSwerveControllerCommand(Trajectories.twoConeFreelane()),
      new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCube)),
      new Wait(0.1),
      new ArmHigh(Constants.ArmConstants.highCubeShoulder, Constants.ArmConstants.highCubeElbow),
      new InstantCommand(() -> m_Arm.clawTo(0))

        
  );
    super.setInitialPose(Trajectories.twoConeFreelane());
    new Stop();
  }
}

