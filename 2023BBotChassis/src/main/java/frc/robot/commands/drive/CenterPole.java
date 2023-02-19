
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterPole extends PIDCommand {

  SwerveDrive m_Drive;
  Vision m_Vision;


  public CenterPole(Vision vision, SwerveDrive swerve, XboxController driver) {
      super(
          // The controller that the command will use
          new PIDController(4, 0, 0.00),
          // This should return the measurement
          () -> vision.getTx(),
          // This should return the setpoint (can also be a constant)
          0,
          // This uses the output
          output -> {
            swerve.drive(new Translation2d(-(driver.getLeftY()*Constants.Swerve.maxSpeed), output), 0, true, false, false);
          });

      getController().enableContinuousInput(-1, 1);
      getController().setTolerance(0.05, 1);
      m_Drive = SwerveDrive.getInstance();
      m_Vision = Vision.getInstance();
      addRequirements(m_Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint() || !m_Vision.getHasTarget();
  }
}
