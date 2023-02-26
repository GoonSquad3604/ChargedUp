// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReadyToRecieve extends SequentialCommandGroup {

  double shoulderClicks;
  Shoulder shoulder;

  /** Creates a new ReadyToRecieve. */
  public ReadyToRecieve() {
    shoulder = Shoulder.getInstance();
    shoulderClicks = shoulder.getShoulderClicks();
    if(shoulderClicks > 290.0) {
      addCommands(
        new ElbowTo(245.101504),
        new ShoulderTo(247.958593),
        new ElbowTo(201.261055)  
      );
    }

  }
}
