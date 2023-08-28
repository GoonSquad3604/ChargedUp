// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  private DriveTrain s_DriveTrain;

  private DoubleSupplier leftDriveSupplier;
  private DoubleSupplier rightDriveSupplier;
  private DoubleSupplier slower;
  private double speed = 1.0;


  public DefaultDrive(DoubleSupplier leftDriveSupplier, DoubleSupplier rightDriveSupplier) {
    s_DriveTrain = DriveTrain.getInstance();
    
    this.leftDriveSupplier = leftDriveSupplier;
    this.rightDriveSupplier = rightDriveSupplier;
    this.slower = slower;

    addRequirements(s_DriveTrain);
  }

  //Called every time the scheduler runs while the command is scheduled.
 
@Override
  public void execute() {
  //   if(slower.getAsDouble() > .5) {
  //     speed = 0.25;
  //   }
  //   else speed = 1;
  //   double leftVal = MathUtil.applyDeadband(leftDriveSupplier.getAsDouble(), Constants.General.deadband)*speed;
  
  //   double rightVal = MathUtil.applyDeadband(rightDriveSupplier.getAsDouble(), Constants.General.deadband)*speed;
  
  // s_DriveTrain.setLeftDrivepower(leftVal);

  // s_DriveTrain.setRightDrivepower(rightVal);
    s_DriveTrain.setLeftDrivepower(MathUtil.applyDeadband(leftDriveSupplier.getAsDouble(), Constants.General.deadband));

    s_DriveTrain.setRightDrivepower(MathUtil.applyDeadband(rightDriveSupplier.getAsDouble(), Constants.General.deadband));
  
  }
  
}
