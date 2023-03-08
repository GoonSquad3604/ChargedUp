// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends PIDCommand {
  private SwerveDrive m_Drive;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive drive) {
    super(
        // The controller that the command will use
        new PIDController(5.0, 0, 0),
        // This should return the measurement
        () -> drive.getRoll(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drive.drive(new Translation2d(output, 0), 0, true, false, false);
        });
      //getController().enableContinuousInput(-1, 1);
      getController().setTolerance(3, 1);
      m_Drive = SwerveDrive.getInstance();
      addRequirements(m_Drive);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
