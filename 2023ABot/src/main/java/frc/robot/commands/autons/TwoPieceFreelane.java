// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.arm.ArmHighCube;
import frc.robot.commands.arm.HomeFromReady;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.ReadyToRecieve;
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
public class TwoPieceFreelane extends GoonAutonCommand{

    LED m_Led;
    Arm m_Arm;
    Intake m_Intake;
  
    public TwoPieceFreelane(LED led, Intake intake){
      m_Led = led;
      m_Intake = intake;
      m_Arm = Arm.getInstance();
      super.addCommands(
        new InstantCommand(() -> m_Arm.setStartingPos()),
        new SetConeMode(m_Led),
        new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCone)),
        new Wait(.25),
        new ArmHigh(),
        AutonUtils.getSwerveControllerCommand(Trajectories.freeLaneMeterBack()),
        new Wait(0.5),
        new InstantCommand(() -> m_Arm.clawTo(0)),
        new Wait(0.25),
        new HomePosition(),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new Wait(1),
            AutonUtils.getSwerveControllerCommand(Trajectories.twoPieceFreeLane())),
          new SequentialCommandGroup(
            
            new SetCubeMode(m_Led),
            new Wait(.5),
            new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeDown)),
            new InstantCommand(() -> m_Intake.toggle()),
            new InstantCommand(() -> m_Intake.runIntake()),
            //new Wait(1),
            new ReadyToRecieve(),
            new Wait(.5),
            new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCube)),
            new Wait(.25),
            new InstantCommand(() -> m_Intake.stopIntake()),
            new InstantCommand(() -> m_Intake.hingeTo(0)),
            new InstantCommand(() -> m_Intake.toggle()),
            new ArmHighCube(),
            new Wait(1)
          )
          // new InstantCommand(() -> m_Arm.clawTo(0))
        ),
        new InstantCommand(() -> m_Arm.clawTo(0)),
        new Stop()
    );
      super.setInitialPose(Trajectories.freeLaneMeterBack());
      
    }
  }
