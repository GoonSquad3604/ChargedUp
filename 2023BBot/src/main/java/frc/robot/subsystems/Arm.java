// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax shoulder;
  private CANSparkMax elbow;
  private static Arm _instance;
  
  /** Creates a new Arm. */
  public Arm() {
    shoulder = new CANSparkMax(Constants.ArmConstants.shoulderID, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
  }

  public static final Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;
  }

  public void setShoulder(double power) {
    shoulder.set(power);

  }
  public void setElbow(double power) {
    elbow.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
