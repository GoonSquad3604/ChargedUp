// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class FastAutoBalance extends CommandBase {
  SwerveDrive m_drive;
  /** Creates a new FastAutoBalance. */
  public FastAutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = SwerveDrive.getInstance();
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_drive.getRoll()) > 12.5) {
      m_drive.drive(new Translation2d(m_drive.getRoll()*0.05,0), 0, true, false, false);
    }
    else {
      m_drive.drive(new Translation2d(0,0), 0, true, false, false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
