// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {


  private WPI_TalonSRX mainIntake = new WPI_TalonSRX(Constants.IntakeConstants.mainIntakeID);
  private WPI_TalonSRX hinge = new WPI_TalonSRX(Constants.IntakeConstants.hingeID);
  private WPI_TalonSRX bigWheel = new WPI_TalonSRX(Constants.IntakeConstants.bigWheelID);

  private static Intake _instance;
  
  
  public Intake()  {
    mainIntake.setNeutralMode(NeutralMode.Coast);
    hinge.setNeutralMode(NeutralMode.Coast);
    bigWheel.setNeutralMode(NeutralMode.Coast);
  }

  public static Intake getInstance() {
    if(_instance == null) {
      _instance = new Intake();
    }
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
