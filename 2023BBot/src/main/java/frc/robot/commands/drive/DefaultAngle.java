// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultAngle extends PIDCommand {

  SwerveDrive m_Drive;

  public DefaultAngle(SwerveDrive swerve) {

    super(
        // The controller that the command will use
        new PIDController(0.108, 0, 0.00),
        // This should return the measurement
        () -> Math.abs(swerve.getPose().getRotation().getDegrees()),
        // This should return the setpoint (can also be a constant)
        12,
        // This uses the output
        output -> {
          swerve.drive(new Translation2d(0,0), output, true, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);

    getController()
        .setTolerance(6, 1);
    m_Drive = SwerveDrive.getInstance();
    addRequirements(m_Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
