package frc.robot.commands.autons;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.arm.HomeFromReady;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.intake.IntakeUntilPickup;
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
public class OnePieceMid extends GoonAutonCommand{

  LED m_Led;
  Arm m_Arm;
  Intake m_Intake;
  HashMap<String, Command> eventMap;


  public OnePieceMid(LED led, Intake intake){
    eventMap = new HashMap<String, Command>();

    eventMap.put("IntakeShooterPos", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeShoot)));
    eventMap.put("IntakeDown", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeDown)));
    eventMap.put("RunIntake", new IntakeUntilPickup());
    eventMap.put("HomePos", new HomePosition());
    
    m_Led = led;
    m_Intake = intake;
    m_Arm = Arm.getInstance();
    super.addCommands(
      new SetConeMode(m_Led),
      new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCone)),
      new ArmHigh(),
      new Wait(.5),
      AutonUtils.getPathWithEvents(Trajectories.midMeterBack(), eventMap),
      new Wait(.5),
      new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.startingPos)),
      new Wait(.5),
      new InstantCommand(() -> m_Arm.stopClaw()),
      new Stop()
  );
    super.setInitialPose(Trajectories.midMeterBack());
  }
}
