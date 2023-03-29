// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.time.Instant;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.HomeFromReady;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.utils.Wait;
import frc.robot.subsystems.Intake;

public class ToggleHinge extends InstantCommand {
  Intake m_Intake;
  public ToggleHinge(Intake intake) {
    //m_Intake = Intake.getInstance();
    m_Intake = intake;
    
    
  }

  @Override
  public void initialize() {
    if(m_Intake.getEncoder() > Constants.IntakeConstants.hingeUp + .03 ){

      m_Intake.hingeTo(Constants.IntakeConstants.hingeUp);

    }
    else {
      m_Intake.hingeTo(Constants.IntakeConstants.hingeDown);
    }
  }
}
