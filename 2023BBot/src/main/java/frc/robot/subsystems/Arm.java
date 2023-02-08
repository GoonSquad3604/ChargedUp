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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax shoulder;
  private CANSparkMax elbow;
  private Encoder shoulderEncoder;
  private Encoder elbowEncoder;

  private static Arm _instance;
  //private SparkMaxAlternateEncoder encoder;
  
  /** Creates a new Arm. */
  public Arm() {
    shoulder = new CANSparkMax(Constants.ArmConstants.shoulderID, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    //encoder = elbow.getAlternateEncoder(, 0);
  }

  public static double[] getAngle(double x, double y) {
    final double a = Constants.ArmConstants.bottomArmLength;
    final double b = Constants.ArmConstants.topArmLength;
    double elbow = Math.acos((Math.pow(x, 2) + Math.pow(y,2)-Math.pow(a, 2) - Math.pow(b, 2))/(2*a*b) );
    double shoulder = Math.atan(y/x) + Math.atan((b*Math.sin(elbow))/(a + b*Math.cos(elbow)));
   
    double angles[] = {Math.toDegrees(elbow), Math.toDegrees(shoulder)};
    return angles;
}

public static void main(String[] args) {
    double wantedAngle[] = getAngle(2,2);
    System.out.println(wantedAngle[1]);
    System.out.println(wantedAngle[0]);
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

  public void stopShoulder() {
    shoulder.set(0);
  }

  public void stopElbow() {
    elbow.set(0);
  }

  public void elbowEncoder() {
    elbow.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
