// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;


public class SwerveDefaultDrive extends CommandBase {
  /** Creates a new SwerveDefaultDrive. */

  private SwerveDrive s_Swerve;    
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier rotSupplier;
  private BooleanSupplier rotLocatSupplier;

  public SwerveDefaultDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, BooleanSupplier rotLocSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = SwerveDrive.getInstance();

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.rotLocatSupplier = rotLocSupplier;

    addRequirements(s_Swerve);


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationVal = MathUtil.applyDeadband(xSupplier.getAsDouble()*-1, Constants.General.deadband);
    double strafeVal = MathUtil.applyDeadband(ySupplier.getAsDouble()*-1, Constants.General.deadband);
    double rotationVal = MathUtil.applyDeadband(rotSupplier.getAsDouble()*-1, Constants.General.deadband);

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity,  
        true,
        rotLocatSupplier.getAsBoolean()
    );
  }

}
