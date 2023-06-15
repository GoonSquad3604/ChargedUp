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

  private CANSparkMax elbow;
  private CANSparkMax roller;

  private static Arm _instance;
  private AbsoluteEncoder elbowEncoder;
  
  private SparkMaxPIDController elbow_pidController;

  private static final int kCPR = 8192;
  /** Makes a new Arm */
  public Arm() {
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    // roller = 
    elbow.restoreFactoryDefaults();
   
    roller.restoreFactoryDefaults();

    
    elbow.setInverted(true);

     //Arm encoders
     elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
     elbowEncoder.setInverted(false);

     elbowEncoder.setZeroOffset(45/360);

      // Elbow PID
    elbow_pidController = elbow.getPIDController();
    elbow_pidController.setFeedbackDevice(elbowEncoder);
    elbow_pidController.setP(Constants.ArmConstants.elbowUpP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
    
    elbow_pidController.setOutputRange(-1.0, 1.0);

    // Brake mode
    elbow.setIdleMode(IdleMode.kBrake);
    elbow.setClosedLoopRampRate(.25);

  }

  public static Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();

    }
    return _instance;
  }

  public void setElbow(double power) {
    elbow.set(power * .7);
  }
  
  public void stopElbow() {
    elbow.set(0);
  }

  public double getElbowClicks() {
    return elbowEncoder.getPosition()*360;
  }

  public void spinRollers(double power) {
    roller.set(power);
  }

  public void reverseRollers(double power) {
    roller.set(-power);
  }
  
  public void stopClaw(){
    roller.set(0);
  }

  public void elbowTo(double refrence){
    elbow_pidController.setReference(refrence/360, CANSparkMax.ControlType.kPosition);
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

  public void notReadyToRecieve(){
    isReadyToRecieve = false;
  }

  public boolean isReadyToRecieve(){
    return isReadyToRecieve;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elbowEncoder", getElbowClicks());
  }
}
