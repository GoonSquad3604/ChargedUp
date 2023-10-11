// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private WPI_TalonFX shoot = new WPI_TalonFX(Constants.ShooterConstants.shootID);

  private CANSparkMax Turret = new CANSparkMax(Constants.ShooterConstants.turretId, MotorType.kBrushless);
  private static Shooter _instance;
  /** Creates a new Shooter. */

  public Shooter() {
    shoot.setNeutralMode(NeutralMode.Coast);
    Turret.setIdleMode(IdleMode.kBrake);
  }

  public static Shooter getInstance() {
    if(_instance == null) {
      _instance = new Shooter();
    }
    return _instance;
  }

  public void setPower(double power) {
    shoot.set(-power);
  }

  public void setTurretTo(double turretpower){
    Turret.set(turretpower);
  }

  public void turretReverse(double reverse){
    Turret.set(-reverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}





