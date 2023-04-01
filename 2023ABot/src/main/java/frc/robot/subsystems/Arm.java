// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ReadyToRecieve;

public class Arm extends SubsystemBase {

  private boolean isReadyToRecieve = false;

  // private CANSparkMax shoulder1;
  // private CANSparkMax shoulder2;
  private CANSparkMax elbow;
  private CANSparkMax claw;

  private static Arm _instance;
  // private AbsoluteEncoder shoulderEncoder;
  private AbsoluteEncoder elbowEncoder;
  //private RelativeEncoder clawEncoder;
  private AbsoluteEncoder clawEncoder;

  // PIDS!
  private SparkMaxPIDController claw_pidController;
  private SparkMaxPIDController elbow_pidController;

  private RelativeEncoder backupencoder;

  private static final int kCPR = 8192;
  
  /** Creates a new Arm. */
  public Arm() {
    // shoulder1 = new CANSparkMax(Constants.ArmConstants.shoulder1ID, MotorType.kBrushless);
    // shoulder2 = new CANSparkMax(Constants.ArmConstants.shoulder2ID, MotorType.kBrushless);
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    claw = new CANSparkMax(Constants.ArmConstants.clawId, MotorType.kBrushless);
    
    elbow.restoreFactoryDefaults();
    // shoulder1.restoreFactoryDefaults();
    // shoulder2.restoreFactoryDefaults();
    claw.restoreFactoryDefaults();

    // shoulder1.setInverted(true);
    // shoulder2.follow(shoulder1, true);
    elbow.setInverted(true);

    

    //Arm encoders
    // shoulderEncoder = shoulder1.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder.setInverted(false);
    //elbowEncoder.setZeroOffset(180);
    //clawEncoder = claw.getEncoder();

    clawEncoder = claw.getAbsoluteEncoder(Type.kDutyCycle);
    clawEncoder.setInverted(true);


    backupencoder = claw.getEncoder();
    backupencoder.setPosition(0);
    
    //clawEncoder.setPosition(0);
    elbowEncoder.setZeroOffset(45/360);
    // PID Controllers

    // Claw PID
    claw_pidController = claw.getPIDController();
    claw_pidController.setFeedbackDevice(clawEncoder);
    claw_pidController.setP(Constants.ArmConstants.clawP);
    claw_pidController.setI(Constants.ArmConstants.clawI);
    claw_pidController.setD(Constants.ArmConstants.clawD);
    claw_pidController.setOutputRange(-1.0, 1.0);

    claw.enableSoftLimit(SoftLimitDirection.kReverse, true);
    claw.setSoftLimit(SoftLimitDirection.kReverse, -100);

    // Elbow PID
    elbow_pidController = elbow.getPIDController();
    elbow_pidController.setFeedbackDevice(elbowEncoder);
    elbow_pidController.setP(Constants.ArmConstants.elbowUpP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
    
    elbow_pidController.setOutputRange(-1.0, 1.0);


    // Brake mode
    // shoulder1.setIdleMode(IdleMode.kBrake);
    // shoulder2.setIdleMode(IdleMode.kBrake);
    elbow.setIdleMode(IdleMode.kBrake);
    claw.setIdleMode(IdleMode.kBrake);
    elbow.setClosedLoopRampRate(0.25);

    //reset encoders
    // resetShoulderEncoder();
    //resetElbowEncoder();

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

  public static Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;
  }

  // public void setShoulder(double power) {
  //   shoulder1.set(power * .4);

  // }
  public void setElbow(double power) {
    elbow.set(power *.7);
  }

  // public void stopShoulder() {
  //   shoulder1.set(0);
  // }

  public void stopElbow() {
    elbow.set(0);
  }

  public double getElbowClicks() {
    return elbowEncoder.getPosition()*360;
  }

  // public double getShoulderClicks() {
  //   return shoulderEncoder.getPosition()*360.0;
  // }

  public double getClawClicks() {
    return clawEncoder.getPosition();
  }

  // Claw
  public void moveClaw(double power) {
    claw.set(power);
  }

  public void stopClaw() {
    claw.set(0);
  }

  // public void setStartingPos() {
  //   clawEncoder.setPosition(Constants.ArmConstants.startingPos);
  // }

  public void clawTo(double refrence) {
    claw_pidController.setReference(refrence, ControlType.kPosition);
    
  }

  
  public void elbowTo(double refrence) {
    elbow_pidController.setReference(refrence/360, CANSparkMax.ControlType.kPosition);
  }



  // public void resetShoulderEncoder(){
  //   shoulderEncoder.setZeroOffset(0);
  // }


  public void resetElbowEncoder() {
    elbowEncoder.setZeroOffset(0);
  }

  public void setUpP() {
    elbow_pidController.setP(Constants.ArmConstants.elbowUpP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
  }
  public void setDownP() {
    elbow_pidController.setP(Constants.ArmConstants.elbowDownP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
  }

  public void setReadyToRecieve() {
    isReadyToRecieve = true;
  }

  public void notReadyToRecieve() {
    isReadyToRecieve = false;
  }

  public boolean getReadyToRecieve() {
    return isReadyToRecieve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("ShoulderEncoder", getShoulderClicks());
    SmartDashboard.putNumber("ElbowEncoder", getElbowClicks());
    SmartDashboard.putNumber("ClawEncoder", getClawClicks());
    SmartDashboard.putBoolean("Claw Open", Math.abs(getElbowClicks()) < 5);
    SmartDashboard.putNumber("Backup Claw Encoder", backupencoder.getPosition());
    //SmartDashboard.putNumber("Elbow Velocity", elbowEncoder.getVelocity());
    //SmartDashboard.putNumber("Elbow P", elbow_pidController.getP());
    

    
  }
}
