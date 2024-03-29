// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import org.ejml.dense.fixed.MatrixFeatures_DDF2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.arm.ArmHighCube;
import frc.robot.commands.arm.ElbowTo;
import frc.robot.commands.arm.HomeFromReady;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.arm.ShoulderTo;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.states.SetConeMode;
import frc.robot.commands.states.SetCubeMode;
import frc.robot.commands.utils.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.util.auton.AutonUtils;
import frc.robot.util.auton.GoonAutonCommand;
import frc.robot.util.auton.Trajectories;

/** Add your docs here. */
public class AutonTest extends GoonAutonCommand{

  LED m_Led;
  Arm m_Arm;
  Intake m_Intake;

  public AutonTest(LED led, Intake intake){
    m_Led = led;
    m_Intake = intake;
    m_Arm = Arm.getInstance();
    super.addCommands(
      AutonUtils.getSwerveControllerCommand(Trajectories.TestPath())
  );
    super.setInitialPose(Trajectories.TestPath());
    
  }
}

