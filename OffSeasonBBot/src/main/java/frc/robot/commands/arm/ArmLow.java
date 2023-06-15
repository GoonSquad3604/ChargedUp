// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.StateController;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLow extends ParallelCommandGroup {
  StateController stateController;
  /** Creates a new ArmLow. */
  public ArmLow() {
    stateController = StateController.getInstance();
    addCommands(new ShoulderTo(Constants.ArmConstants.hybridShoulder), new ElbowTo(Constants.ArmConstants.hybridElbow));
  }
}
