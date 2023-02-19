// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private Arm s_Arm;
  private DoubleSupplier shoulderSupplier;
  private DoubleSupplier elbowSupplier;
  public ArmDefaultCommand(DoubleSupplier shoulderSupplier, DoubleSupplier elbowSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Arm = Arm.getInstance();

    this.shoulderSupplier = shoulderSupplier;
    this.elbowSupplier = elbowSupplier;
    addRequirements(s_Arm);
  
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    s_Arm.setShoulder(MathUtil.applyDeadband(shoulderSupplier.getAsDouble(), Constants.General.deadband));
    s_Arm.setElbow(MathUtil.applyDeadband(elbowSupplier.getAsDouble(), Constants.General.deadband));
  }


}
