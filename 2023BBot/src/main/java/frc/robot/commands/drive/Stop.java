// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stop extends InstantCommand {
  
  private SwerveDrive m_Drive;

  public Stop() {
    m_Drive = SwerveDrive.getInstance();
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drive.drive(new Translation2d(0,0), 0, false, false);
  }
}
