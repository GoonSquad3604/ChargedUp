package frc.robot.commands.autons;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
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
public class ThreeCubeAuton extends GoonAutonCommand{

  LED m_Led;
  Arm m_Arm;
  Intake m_Intake;

  HashMap<String, Command> eventMap;

  public ThreeCubeAuton(LED led, Intake intake){
    eventMap = new HashMap<String, Command>();

    eventMap.put("IntakeShooterPos", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeShoot)));
    eventMap.put("IntakeDown", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeDown)));
    eventMap.put("RunIntake", new IntakeUntilPickup());

    m_Led = led;
    m_Intake = intake;
    m_Arm = Arm.getInstance();
    super.addCommands(
      new SetCubeMode(m_Led),
      new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeShoot)),
      new Wait(0.5),
      new InstantCommand(() -> m_Intake.runIntake(Constants.IntakeConstants.vomitSpeed)),
      new Wait(.35),
      new InstantCommand(() -> m_Intake.stopIntake()),
      AutonUtils.getPathWithEvents(Trajectories.ThreeCubePurpleCannon_1(), eventMap),
      new Stop()
  );
    super.setInitialPose(Trajectories.ThreeCubePurpleCannon_1());
  }
}
