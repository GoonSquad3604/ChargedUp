// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomePositionCone extends ParallelCommandGroup {

  double shoulderClicks;
  Shoulder shoulder;
  Arm m_Arm;

  /** Creates a new HomePositionCone. */
  public HomePositionCone() {
    shoulder = Shoulder.getInstance();
    shoulderClicks = shoulder.getShoulderClicks();
    m_Arm = Arm.getInstance();
      addCommands(
        new ElbowTo(Constants.ArmConstants.homeElbow),
        new ShoulderTo(Constants.ArmConstants.homeShoulder),
        new InstantCommand(() -> m_Arm.notReadyToRecieve())
      );
  }
}