// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

public class HomeFromReady extends InstantCommand {

  Arm m_Arm;
  Shoulder m_Shoulder;

  public HomeFromReady() {
    m_Arm = Arm.getInstance();
    m_Shoulder = Shoulder.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.elbowTo(Constants.ArmConstants.homeElbow);
    m_Shoulder.shoulderTo(Constants.ArmConstants.homeShoulder);
  }
}
