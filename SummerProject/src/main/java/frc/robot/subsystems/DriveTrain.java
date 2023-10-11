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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  DifferentialDrive diffDrive;
  MotorControllerGroup leftGroup;
  MotorControllerGroup rightGroup;

  double powerInput; 
  double rotInput;

  

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
    
    leftGroup = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightGroup = new MotorControllerGroup(rightDrive1, rightDrive2);
    leftGroup.setInverted(true);
    diffDrive = new DifferentialDrive(leftGroup, rightGroup);

  } 

  public static DriveTrain getInstance() {
    if(_instance == null) {
      _instance = new DriveTrain();

    }
    return _instance;

  }

  public void drive(double power, double rotation) {
    diffDrive.arcadeDrive(power, rotation);
    powerInput = power;
    rotInput = rotation;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Power", powerInput);
    SmartDashboard.putNumber("rotation", rotInput);
  }
}
