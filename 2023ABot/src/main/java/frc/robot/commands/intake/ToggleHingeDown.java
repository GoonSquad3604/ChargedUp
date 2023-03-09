// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.HomeFromReady;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.utils.Wait;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleHingeDown extends SequentialCommandGroup {
  Intake m_Intake;
  /** Creates a new ToggleHingeDown. */
  public ToggleHingeDown(Intake intake) {
    m_Intake = intake;

      addCommands(
        new HomeFromReady(),
        new Wait(1),
        new InstantCommand(() -> m_Intake.hingeTo(0)),
        new InstantCommand(() -> m_Intake.toggle())
      );
  }
}
