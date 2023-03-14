// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;

public class ElbowTo extends CommandBase {

  Arm m_Arm;
  Shoulder m_Shoulder;
  double m_refrence;
  boolean goodToMove = false;
  boolean ended = false;

  public ElbowTo(double refrence) {
    m_Arm = Arm.getInstance();
    m_Shoulder = Shoulder.getInstance();
    m_refrence = refrence;
    ended = false;
    goodToMove = false;

    addRequirements(m_Arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goodToMove = false;
    //SmartDashboard.putNumber("Elbow refrence", m_refrence);
    if(m_Arm.getElbowClicks() > m_refrence) {
      m_Arm.setUpP();
    }
    else {
      m_Arm.setDownP();

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putString("Elbow to status", "running " + m_refrence);
   // SmartDashboard.putString("Shoulder is over 280", "no");
    if(m_Shoulder.getShoulderClicks() > 153.0) {
     // SmartDashboard.putString("Shoulder is over 280", "yep");
      goodToMove = true;
    }
    
    if(goodToMove) {
      m_Arm.elbowTo(m_refrence);
    }
    else{
      m_Arm.setElbow(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // SmartDashboard.putString("Elbow to status", "ended");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putString("Elbow to status", "running " + m_refrence);
    return (Math.abs(m_Arm.getElbowClicks()-m_refrence) < 10);
  }
}
