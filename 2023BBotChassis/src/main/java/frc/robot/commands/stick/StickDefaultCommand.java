// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stick;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Stick;

public class StickDefaultCommand extends CommandBase {
  /** Creates a new StickDefaultCommand. */
  private DoubleSupplier stickSupplier;
  private Stick s_Stick;


  public StickDefaultCommand(DoubleSupplier stickSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Stick = Stick.getInstance();
    this.stickSupplier = stickSupplier;
    addRequirements(s_Stick);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Stick.moveStick(MathUtil.applyDeadband(stickSupplier.getAsDouble(),Constants.General.deadband ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
