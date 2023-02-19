// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Intake arms that move up and down
  CANSparkMax leftHinge;
  CANSparkMax rightHinge;

  // Spaghetti wheels
  CANSparkMax leftIntake;

  public Intake() {

    // Hinges
    leftHinge = new CANSparkMax(Constants.IntakeConstants.leftHingeId, MotorType.kBrushless);
    rightHinge = new CANSparkMax(Constants.IntakeConstants.rightHingeId, MotorType.kBrushless);
    rightHinge.follow(leftHinge);

    // Spaghetti
    leftIntake = new CANSparkMax(Constants.IntakeConstants.leftIntakeId, MotorType.kBrushless);
  }

  public void setHinge(double power) {
    leftHinge.set(power);
  }

  public void runCubeIntake() {
    leftIntake.set(Constants.IntakeConstants.cubeSpeed);
  }

  public void runConeIntake() {
    leftIntake.set(Constants.IntakeConstants.coneSpeed);
  }

  public void stopIntake() {
    leftIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
