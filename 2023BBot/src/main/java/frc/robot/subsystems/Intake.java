// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Intake arms that move up and down
  CANSparkMax leftHinge;
  CANSparkMax rightHinge;
  StateController stateController;
  private static Intake _instance;

  // Spaghetti wheels
  WPI_TalonSRX intake;

  public Intake() {

    stateController = StateController.getInstance();

    // Hinges
    leftHinge = new CANSparkMax(Constants.IntakeConstants.leftHingeId, MotorType.kBrushless);
    rightHinge = new CANSparkMax(Constants.IntakeConstants.rightHingeId, MotorType.kBrushless);
    
    leftHinge.restoreFactoryDefaults();
    rightHinge.restoreFactoryDefaults();
    rightHinge.follow(leftHinge, true);

    //rightHinge.setInverted(true);
    rightHinge.setIdleMode(IdleMode.kBrake);
    leftHinge.setIdleMode(IdleMode.kBrake);

    // Spaghetti
    intake = new WPI_TalonSRX(Constants.IntakeConstants.intakeId);
  }

  public static final Intake getInstance() {
    if (_instance == null) {
            _instance = new Intake();
    }
    return _instance;
  }

  public void setHinge(double leftPower, double rightPower) {
    leftHinge.set(leftPower);
    rightHinge.set(rightPower);

  }

  public void runIntake() {
    intake.set(stateController.intakeSpeed);
  }

  public void stopIntake() {
    intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftintake setting", leftHinge.get());
    SmartDashboard.putNumber("rightintake setting", rightHinge.get());
  }
}
