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
  private CANSparkMax claw;

  private static Arm _instance;
  private AbsoluteEncoder elbowEncoder;
  private AbsoluteEncoder clawEncoder;

  // PIDS!
  private SparkMaxPIDController claw_pidController;
  private SparkMaxPIDController elbow_pidController;

  private RelativeEncoder backupencoder;

  private static final int kCPR = 8192;
  
  /** Creates a new Arm. */
  public Arm() {
    
    elbow = new CANSparkMax(Constants.ArmConstants.elbowID, MotorType.kBrushless);
    claw = new CANSparkMax(Constants.ArmConstants.clawId, MotorType.kBrushless);
    
    elbow.restoreFactoryDefaults();
   
    claw.restoreFactoryDefaults();

    
    elbow.setInverted(true);

    

    //Arm encoders
    elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
    elbowEncoder.setInverted(false);

    clawEncoder = claw.getAbsoluteEncoder(Type.kDutyCycle);
    clawEncoder.setInverted(true);


    backupencoder = claw.getEncoder();
    backupencoder.setPosition(0);
    
    elbowEncoder.setZeroOffset(45/360);
    // PID Controllers

    // Claw PID
    claw_pidController = claw.getPIDController();
    claw_pidController.setFeedbackDevice(clawEncoder);
    claw_pidController.setP(Constants.ArmConstants.clawP);
    claw_pidController.setI(Constants.ArmConstants.clawI);
    claw_pidController.setD(Constants.ArmConstants.clawD);
    claw_pidController.setOutputRange(-1.0, 1.0);

    

    // Elbow PID
    elbow_pidController = elbow.getPIDController();
    elbow_pidController.setFeedbackDevice(elbowEncoder);
    elbow_pidController.setP(Constants.ArmConstants.elbowUpP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
    
    elbow_pidController.setOutputRange(-1.0, 1.0);


    // Brake mode
    
    elbow.setIdleMode(IdleMode.kBrake);
    claw.setIdleMode(IdleMode.kBrake);
    elbow.setClosedLoopRampRate(0.25);

    //reset encoders
   


  }

  public static Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;
  }

  
/**
 * Sets elbow to a power.
 * @param power power of elbow
 */
  public void setElbow(double power) {
    elbow.set(power *.7);
  }

/**
 * Stops elbow.
 */
  public void stopElbow() {
    elbow.set(0);
  }
/**
 * @return Elbow Encoder in Degrees
 */
  public double getElbowClicks() {
    return elbowEncoder.getPosition()*360;
  }

  /**
   * Gets Claw Encoder Position.
   */
  public double getClawClicks() {
    return clawEncoder.getPosition();
  }

  // Claw

  /**
   * @param power Power of Claw
   */
  public void moveClaw(double power) {
    claw.set(power);
  }

  public void stopClaw() {
    claw.set(0);
  }

 /**
  * Uses PID looping to set the claw to a certain position 
  * @param refrence Position to send claw to 
  */
  public void clawTo(double refrence) {
    claw_pidController.setReference(refrence, ControlType.kPosition);
    
  }

  /**
   * Uses PID looping to set the elbow to a certain position 
   * @param refrence Position to send elbow to 
   */
  public void elbowTo(double refrence) {
    elbow_pidController.setReference(refrence/360, CANSparkMax.ControlType.kPosition);
  }


/**
 * Sets the PID values for elbow going up
 */
  public void setUpP() {
    elbow_pidController.setP(Constants.ArmConstants.elbowUpP);
    elbow_pidController.setI(Constants.ArmConstants.elbowI);
    elbow_pidController.setD(Constants.ArmConstants.elbowD);
  }
  /**
   * Sets the PID value for the elbow when it goes down
   */
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
/**
 * @return if the robot is in the ready to recieve position
 */
  public boolean getReadyToRecieve() {
    return isReadyToRecieve;
  }
/**
*puts the encoder positions for the elbow and the claw on the smart dashboard
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElbowEncoder", getElbowClicks());
    SmartDashboard.putNumber("ClawEncoder", getClawClicks());
    SmartDashboard.putNumber("claw current", claw.getOutputCurrent());
    
    
  }
}
