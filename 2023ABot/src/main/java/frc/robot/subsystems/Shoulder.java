// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
  private CANSparkMax shoulder1;
  private CANSparkMax shoulder2;
  private AbsoluteEncoder shoulderEncoder;
  private SparkMaxPIDController pidController;
  private static Shoulder _instance;
  private boolean refrenceSet;

  /** Creates a new Shoulder. */
  public Shoulder() {
    shoulder1 = new CANSparkMax(Constants.ArmConstants.shoulder1ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.ArmConstants.shoulder2ID, MotorType.kBrushless);

    shoulder1.restoreFactoryDefaults();
    shoulder2.restoreFactoryDefaults();

    shoulder1.setInverted(true);
    shoulder2.follow(shoulder1, true);

    shoulderEncoder = shoulder1.getAbsoluteEncoder(Type.kDutyCycle);

    shoulder1.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);

    resetShoulderEncoder();

    // PID!
    pidController = shoulder1.getPIDController();
    pidController.setFeedbackDevice(shoulderEncoder);

    pidController.setP(Constants.ArmConstants.shoulderUpP);
    pidController.setI(Constants.ArmConstants.shoulderI);
    pidController.setD(Constants.ArmConstants.shoulderD);
    pidController.setOutputRange(-1.0, 1.0);

    refrenceSet = false;
  }

  public static Shoulder getInstance() {
    if(_instance == null) {
      _instance = new Shoulder();
    }
    return _instance;
  }

  public void shoulderTo(double refrence) {
    pidController.setReference(refrence/360.0, CANSparkMax.ControlType.kPosition);
    refrenceSet = true;
  }

  public void setShoulder(double power) {
      shoulder1.set(power * .4);
      refrenceSet = false;
  }
  public void stopShoulder() {
    shoulder1.set(0);
    refrenceSet = false;
  }
  public double getShoulderClicks() {
    return shoulderEncoder.getPosition()*360.0;
  }
  public void resetShoulderEncoder(){
    shoulderEncoder.setZeroOffset(0);
  }

  public void setReferenceBoolean(boolean input){
    refrenceSet = true;
  }

  public void setUpP() {
    pidController.setP(Constants.ArmConstants.shoulderUpP);
    pidController.setI(Constants.ArmConstants.shoulderI);
    pidController.setD(Constants.ArmConstants.shoulderD);
  }
  public void setDownP() {
    pidController.setP(Constants.ArmConstants.shoulderDownP);
    pidController.setI(Constants.ArmConstants.shoulderI);
    pidController.setD(Constants.ArmConstants.shoulderD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShoulderEncoder", getShoulderClicks());

  }
}
