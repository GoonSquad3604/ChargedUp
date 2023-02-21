// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax shoulder1;
  private CANSparkMax shoulder2;
  private CANSparkMax elbow;
  private CANSparkMax claw;

  private static Arm _instance;
  private RelativeEncoder shoulderEncoder;
  private RelativeEncoder elbowEncoder;

  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;
  
  /** Creates a new Arm. */
  public Arm() {
    shoulder1 = new CANSparkMax(Constants.ArmConstants.shoulder1ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.ArmConstants.shoulder2ID, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    claw = new CANSparkMax(Constants.ArmConstants.clawId, MotorType.kBrushless);
    
    elbow.restoreFactoryDefaults();
    shoulder1.restoreFactoryDefaults();
    shoulder2.restoreFactoryDefaults();
    claw.restoreFactoryDefaults();
    shoulder2.follow(shoulder1, true);

    //Arm encoders
    shoulderEncoder = shoulder1.getAlternateEncoder(kAltEncType, kCPR);
    elbowEncoder = elbow.getAlternateEncoder(kAltEncType, kCPR);
    
    // Brake mode
    shoulder1.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);
    elbow.setIdleMode(IdleMode.kBrake);

    

    //encoder = elbow.getAlternateEncoder(, 0);

    //shoulderEncoder = shoulder1.getAlternateEncoder(com.revrobotics.SparkMaxAlternateEncoder.Type.kQuadrature, 0)
  }

  public static double[] getAngle(double x, double y) {
    final double a = Constants.ArmConstants.bottomArmLength;
    final double b = Constants.ArmConstants.topArmLength;
    double elbow = Math.acos((Math.pow(x, 2) + Math.pow(y,2)-Math.pow(a, 2) - Math.pow(b, 2))/(2*a*b) );
    double shoulder = Math.atan(y/x) + Math.atan((b*Math.sin(elbow))/(a + b*Math.cos(elbow)));
   
    double angles[] = {Math.toDegrees(elbow), Math.toDegrees(shoulder)};
    return angles;
  }

  public static final Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;
  }

  public void setShoulder(double power) {
    shoulder1.set(power * .4);

  }
  public void setElbow(double power) {
    elbow.set(power *.7);
  }

  public void stopShoulder() {
    shoulder1.set(0);
  }

  public void stopElbow() {
    elbow.set(0);
  }

  public double getElbowClicks() {
    return elbowEncoder.getPosition();

  }

  public double getShoulderClicks() {
    return shoulderEncoder.getPosition();
  }

  // Claw
  public void moveClaw(double power) {
    claw.set(power);
  }

  public void stopClaw() {
    claw.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShoulderEncoder", getShoulderClicks());
    SmartDashboard.putNumber("ElbowEncoder", getElbowClicks());
    SmartDashboard.putNumber("Elbow Velocity", elbowEncoder.getVelocity());
    
  }
}
