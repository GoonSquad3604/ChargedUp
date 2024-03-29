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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceMidBalanceCube extends GoonAutonCommand{

  LED m_Led;
  Arm m_Arm;
  Intake m_Intake;

  public OnePieceMidBalanceCube(LED led, Intake intake){
    m_Led = led;
    m_Intake = intake;
    m_Arm = Arm.getInstance();
    super.addCommands(
      new SetCubeMode(m_Led),
      new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCube)),
      new ArmHighCube(),
      AutonUtils.getSwerveControllerCommand(Trajectories.midMeterBack()),
      new Wait(0.5),
      new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.startingPos)),
      new Wait(0.25),
      new ParallelCommandGroup(AutonUtils.getSwerveControllerCommand(Trajectories.ontoPlatform()),
        new SequentialCommandGroup(
          new Wait(0.25),
          new HomeFromReady()
        )
      ),
      new AutoBalance(),
      
      new Stop()
  );
    super.setInitialPose(Trajectories.midMeterBack());
  }
}
