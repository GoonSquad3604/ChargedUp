// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ToggleHinge extends InstantCommand {
  Intake m_Intake;
  public ToggleHinge() {
    m_Intake = Intake.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_Intake.toggledUp) {
      m_Intake.hingeTo(Constants.IntakeConstants.hingeDown);
      m_Intake.toggledUp = false;
    }
    else {
      m_Intake.hingeTo(0);
      m_Intake.toggledUp = true;
    }
  }
}
