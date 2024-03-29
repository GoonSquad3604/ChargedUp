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
import frc.robot.commands.drive.AutoBalanceInverse;
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
/** Add your docs here. */
public class TwoPieceFreelaneBalance extends GoonAutonCommand{

    LED m_Led;
    Arm m_Arm;
    Intake m_Intake;

    HashMap<String, Command> eventMap;

    public TwoPieceFreelaneBalance(LED led, Intake intake){
        eventMap = new HashMap<String, Command>();

        eventMap.put("IntakeShooterPos", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeShoot)));
        eventMap.put("ShootHigh", new InstantCommand(() -> m_Intake.runIntake(Constants.IntakeConstants.topCubeSpeed)));
        eventMap.put("IntakeDown", new InstantCommand(() -> m_Intake.hingeTo(Constants.IntakeConstants.hingeDown)));
        eventMap.put("RunIntake", new IntakeUntilPickup());
        eventMap.put("HomePos", new HomePosition());
        eventMap.put("StopClaw", new InstantCommand(() -> m_Arm.stopClaw()));
        eventMap.put("OpenClaw", new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.startingPos)));

        m_Led = led;
        m_Intake = intake;
        m_Arm = Arm.getInstance();
        super.addCommands(
            new SetConeMode(m_Led),
            new InstantCommand(() -> m_Arm.clawTo(Constants.ArmConstants.closedCone)),
            new Wait(0.3),
            new ArmHigh(),
            AutonUtils.getPathWithEvents(Trajectories.freeLaneMeterBack(), eventMap),
            new SetCubeMode(m_Led),
            AutonUtils.getPathWithEvents(Trajectories.ThreeCubePurpleCannon_1Fast(), eventMap),
            new InstantCommand(() -> m_Intake.runIntake(Constants.IntakeConstants.topCubeSpeed)),
            new Wait(0.45),
            new InstantCommand(() -> m_Intake.stopIntake()),
            AutonUtils.getSwerveControllerCommand(Trajectories.BalanceFromFreelane()),
            new AutoBalanceInverse(),
            new Stop()
        );

        super.setInitialPose(Trajectories.freeLaneMeterBack());
    }
}
