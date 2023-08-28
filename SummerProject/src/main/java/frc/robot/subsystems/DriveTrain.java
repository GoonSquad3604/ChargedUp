// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  CANSparkMax leftDrive1;
  CANSparkMax leftDrive2;
  CANSparkMax rightDrive1;
  CANSparkMax rightDrive2;
  private static DriveTrain _instance;

  public DriveTrain() {
    leftDrive1 = new CANSparkMax(Constants.DriveConstants.leftDrive1Id, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.leftDrive2Id, MotorType.kBrushless);
  
    
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.rightDrive1Id, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.rightDrive2Id, MotorType.kBrushless);
    
    leftDrive1.restoreFactoryDefaults();
    leftDrive2.restoreFactoryDefaults();
    rightDrive1.restoreFactoryDefaults();
    rightDrive2.restoreFactoryDefaults();

    leftDrive1.setInverted(true);
    leftDrive2.follow(leftDrive1, true);

    rightDrive1.setInverted(true);
    rightDrive2.follow(rightDrive1, true);

    leftDrive1.setIdleMode(IdleMode.kBrake);
    leftDrive2.setIdleMode(IdleMode.kBrake);
    rightDrive1.setIdleMode(IdleMode.kBrake);
    rightDrive2.setIdleMode(IdleMode.kBrake);

  } 

  public static DriveTrain getInstance() {
    if(_instance == null) {
      _instance = new DriveTrain();

    }
    return _instance;

  }

  public void setLeftDrivepower(double power){
    leftDrive1.set(power);
  }
  public void setRightDrivepower(double power){
  
    rightDrive1.set(power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
