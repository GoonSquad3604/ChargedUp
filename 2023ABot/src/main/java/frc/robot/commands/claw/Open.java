// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Open extends InstantCommand {
  
  Arm m_Arm;
  StateController m_StateController;
  /** Creates open state for claw.  */
  public Open() {
    m_Arm = Arm.getInstance();
    m_StateController = StateController.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.clawTo(0);
  }
}
