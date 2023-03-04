// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LEDS.SetLedsYellow;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShoulderTo extends InstantCommand {

  Shoulder m_Shoulder;
  double m_refrence;

  public ShoulderTo(double refrence) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Shoulder = Shoulder.getInstance();
    m_refrence = refrence;
    addRequirements(m_Shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //determine direction
    //use direction to set high or low p. you need to call shoulder.setupP should.setdownP
    SmartDashboard.putString("Shoulder to status", "moving to: " + m_refrence);
    if(m_Shoulder.getShoulderClicks() > m_refrence) {
      m_Shoulder.setDownP();
    }
    else {
      m_Shoulder.setUpP();
    }
    m_Shoulder.shoulderTo(m_refrence);
  }
}