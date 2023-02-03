// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stick extends SubsystemBase {
  /** Creates a new Stick. */

  private WPI_TalonSRX stickMotor;
  private static Stick _instance;


  public Stick() {
    stickMotor = new WPI_TalonSRX(6);

  }

  public static final Stick getInstance() {
    if (_instance == null) {
      _instance = new Stick();
    }
    return _instance;
  }
  public void moveStick(double power) {
    stickMotor.set(power*.3);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
