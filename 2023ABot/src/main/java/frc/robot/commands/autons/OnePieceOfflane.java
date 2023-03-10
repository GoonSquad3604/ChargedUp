package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmHigh;
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
public class OnePieceOfflane extends GoonAutonCommand{

    LED m_Led;
    Arm m_Arm;
    Intake m_Intake;
  
    public OnePieceOfflane(LED led, Intake intake){
      m_Led = led;
      m_Intake = intake;
      m_Arm = Arm.getInstance();
      super.addCommands(
        new InstantCommand(() -> m_Arm.setStartingPos()),
        new SetConeMode(m_Led),
        new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCone)),
        new Wait(.25),
        new ArmHigh(),
        AutonUtils.getSwerveControllerCommand(Trajectories.offLaneMeterBack()),
        new Wait(0.5),
        new InstantCommand(() -> m_Arm.clawTo(0)),
        new Wait(1),
        AutonUtils.getSwerveControllerCommand(Trajectories.offLaneBackUp()),
        new Stop()
    );
      super.setInitialPose(Trajectories.offLaneMeterBack());
      
    }
  }
  

