// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmMedium extends ParallelCommandGroup {
  /** Creates a new ArmMedium. */
  Arm m_Arm;
  StateController stateController;
  public ArmMedium() {
    m_Arm = Arm.getInstance();
    addCommands(new InstantCommand(() -> m_Arm.notReadyToRecieve()), new ShoulderTo(Constants.ArmConstants.midConeShoulder), new ElbowTo(Constants.ArmConstants.midConeElbow));
  }
}
