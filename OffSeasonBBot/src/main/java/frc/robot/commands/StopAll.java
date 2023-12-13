// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAll extends InstantCommand {
  Shoulder m_Shoulder;
  Arm m_Arm;
  Intake m_Intake;
  public StopAll() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = Intake.getInstance();
    m_Arm = Arm.getInstance();
    m_Shoulder = Shoulder.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setHinge(0, 0);
    m_Intake.stopIntake();
    m_Arm.setElbow(0);
    m_Shoulder.stopShoulder();
  }
}
