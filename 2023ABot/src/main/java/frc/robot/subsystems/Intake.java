// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Intake arms that move up and down
  CANSparkMax leftHinge;
  CANSparkMax rightHinge;
  AbsoluteEncoder hingEncoder;
  StateController stateController;
  private static Intake _instance;

  private DigitalInput sensor;

  // Toggling
  private boolean toggledUp;
  private SparkMaxPIDController hingePIDController;

  // Spaghetti wheels
  WPI_TalonSRX intake;

  public Intake() {
    stateController = StateController.getInstance();
    toggledUp = true;
    // Hinges
    leftHinge = new CANSparkMax(Constants.IntakeConstants.leftHingeId, MotorType.kBrushless);
    rightHinge = new CANSparkMax(Constants.IntakeConstants.rightHingeId, MotorType.kBrushless);
    
    
    leftHinge.restoreFactoryDefaults();
    rightHinge.restoreFactoryDefaults();
    leftHinge.setInverted(true);
    
    rightHinge.follow(leftHinge, true);

    rightHinge.setIdleMode(IdleMode.kBrake);
    leftHinge.setIdleMode(IdleMode.kBrake);

    //rightHinge.setInverted(true);
    
    

    hingEncoder = leftHinge.getAbsoluteEncoder(Type.kDutyCycle);
    hingEncoder.setInverted(true);

    // PIDS!
    hingePIDController = leftHinge.getPIDController();
    hingePIDController.setFeedbackDevice(hingEncoder);
    hingePIDController.setP(Constants.IntakeConstants.hingeP);
    hingePIDController.setI(Constants.IntakeConstants.hingeI);
    hingePIDController.setD(Constants.IntakeConstants.hingeD);
    hingePIDController.setOutputRange(-0.8, 0.8);
    leftHinge.setClosedLoopRampRate(1.5);


    // Spaghetti
    intake = new WPI_TalonSRX(Constants.IntakeConstants.intakeId);
    intake.enableVoltageCompensation(true);

    sensor = new DigitalInput(Constants.IntakeConstants.sensorId);
  }

  public void toggle() {
    if(toggledUp) {
      toggledUp = false;
      //SmartDashboard.putString("toggle", "down"); 
    }
    else {
      toggledUp = true;
      //SmartDashboard.putString("toggle", "up"); 
    }
  }

  public boolean getToggle() {
    return toggledUp;
  }

  public boolean getNotToggle() {
    return !toggledUp;
  }

  public static Intake getInstance() {
    if (_instance == null) {
            _instance = new Intake();
    }
    return _instance;
  }

  public double getEncoder() {
    return hingEncoder.getPosition();
  }

  public void zeroHinge() {
    //hingEncoder.setPosition(0);
  }

  public void setHinge(double leftPower, double rightPower) {
    leftHinge.set(leftPower);
    //rightHinge.set(rightPower);
  }

  public void runIntake() {
    intake.set(stateController.getIntakeSpeed());
    //intake.set(1.0);
  }
  public void runIntake(double speed) {
    intake.set(speed);
    //intake.set(1.0);
  }
  public void vomit() {
    intake.set(Constants.IntakeConstants.vomitSpeed);
  }
  public void stopIntake() {
    intake.set(0);
  }

  public void hingeTo(double position) {
    hingePIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public boolean getIntakeSensor() {
    return !sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("leftintake setting", leftHinge.get());
    //SmartDashboard.putNumber("rightintake setting", rightHinge.get());
    SmartDashboard.putNumber("Hinge Position", getEncoder());
    SmartDashboard.putBoolean("intake sensor", getIntakeSensor());
  }
}
