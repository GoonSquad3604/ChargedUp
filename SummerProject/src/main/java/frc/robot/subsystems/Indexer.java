// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  private CANSparkMax Indexer1 = new CANSparkMax(Constants.IndexerConstants.indexer1ID, MotorType.kBrushless);
  private CANSparkMax Indexer2 = new CANSparkMax(Constants.IndexerConstants.indexer2ID, MotorType.kBrushless);


  private static Indexer _instance;
  public Indexer() {
    Indexer1.restoreFactoryDefaults();
    Indexer2.restoreFactoryDefaults();

   

    Indexer1.setIdleMode(IdleMode.kCoast);
    Indexer2.setIdleMode(IdleMode.kCoast);
  }

  public static Indexer getInstance() {
    if(_instance == null) {
      _instance = new Indexer();
    }
    return _instance;
  }
  public void moveIndex1(double power){
    Indexer1.set(-power);
    Indexer2.set(power);

  }
  public void moveIndex2(double power){
    Indexer2.set(-power);
  Indexer1.set(power);
      }

  @Override
  public void periodic() {
  }
}
