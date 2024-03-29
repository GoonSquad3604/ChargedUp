// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HingeTo extends PIDCommand {
  Intake m_Intake;
  /** Creates a new HingeTo. */
  public HingeTo(Intake intake, double position) {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        () -> intake.getEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          intake.setHinge(output, output);
        });

        getController().enableContinuousInput(-0.1, 0.1);
        m_Intake = intake;
        addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
