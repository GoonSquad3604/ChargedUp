package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.LEDs.SetLedsPurple;
import frc.robot.commands.LEDs.SetLedsYellow;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetConeMode extends InstantCommand {

  private LED m_Led;
  private StateController stateController;

  public SetConeMode(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Led = led;
    stateController = StateController.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Led.setColor(255, 255, 0);
    stateController.setCone();
  }
}
