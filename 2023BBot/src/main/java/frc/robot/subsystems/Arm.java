// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax shoulder;
  private CANSparkMax elbow;
  private static Arm _instance;
  private SparkMaxAlternateEncoder encoder;
  
  /** Creates a new Arm. */
  public Arm() {
    shoulder = new CANSparkMax(Constants.ArmConstants.shoulderID, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    //encoder = elbow.getAlternateEncoder(, 0);
  }

  public static double[] getArmAngles(int height, int distance) {

    // Angle for elbow
    double elbowAngle = Math.acos((Math.pow(distance, 2) + Math.pow(height, 2) - Math.pow(Constants.ArmConstants.bottomArmLength, 2) - Math.pow(Constants.ArmConstants.topArmLength, 2))/2.0*Constants.ArmConstants.bottomArmLength*Constants.ArmConstants.topArmLength)
    / 2.0*Constants.ArmConstants.bottomArmLength*Constants.ArmConstants.topArmLength;
    // Angle for shoulder
    double shoulderAngle = (Math.atan(height/distance)) + Math.atan((Constants.ArmConstants.topArmLength*Math.sin(elbowAngle))/(Constants.ArmConstants.bottomArmLength + Constants.ArmConstants.topArmLength*Math.cos(elbowAngle)));

    double angles[] = {shoulderAngle, elbowAngle};
    return angles;
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
