// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RobotMode;

public class StateController extends SubsystemBase {

  private static StateController _instance;
  private RobotMode m_Mode;
  private double intakeSpeed;
  private double closedClawPos;
  private double highposElbow;
  private double midposElbow;
  private double highposShoulder;
  private double midposShoulder;


  public static StateController getInstance() {
    SmartDashboard.putString("test", "this");
    if(_instance == null) {
      _instance = new StateController();
    }
    return _instance;
  }

  public StateController() {
    setNeutral();
  }

  public void setCube() {
    SmartDashboard.putString("Cube or Cone Mode", "Cube");
    m_Mode = RobotMode.CUBE;
    intakeSpeed = Constants.IntakeConstants.cubeSpeed;
    highposShoulder = Constants.ArmConstants.highCubeShoulder;
    highposElbow = Constants.ArmConstants.highCubeElbow;
    midposShoulder = Constants.ArmConstants.midCubeShoulder;
    midposElbow = Constants.ArmConstants.midCubeElbow;
    closedClawPos = Constants.ArmConstants.closedCube;

  }
  public void setCone() {
    SmartDashboard.putString("Cube or Cone Mode", "Cone");
    m_Mode = RobotMode.CONE;
    intakeSpeed = Constants.IntakeConstants.coneSpeed;
    highposShoulder = Constants.ArmConstants.highConeShoulder;
    highposElbow = Constants.ArmConstants.highConeElbow;
    midposShoulder = Constants.ArmConstants.midConeShoulder;
    midposElbow = Constants.ArmConstants.midConeElbow;
    closedClawPos = Constants.ArmConstants.closedCone;

  }
  public void setNeutral() {
    SmartDashboard.putString("Cube or Cone Mode", "this");
    m_Mode = RobotMode.NEUTRAL;
    intakeSpeed = 0;
   
    highposShoulder = Constants.ArmConstants.highCubeShoulder;
    highposElbow = Constants.ArmConstants.highCubeElbow;
    midposShoulder = Constants.ArmConstants.midCubeShoulder;
    midposElbow = Constants.ArmConstants.midCubeElbow;
    closedClawPos = Constants.ArmConstants.closedCube;
  }
  public RobotMode getMode() {
    return m_Mode;
  }

  public double getIntakeSpeed() {
    return intakeSpeed;
  }

  public double getHighPosElbow() {
    return highposElbow;
  }
  public double getHighPosShoulder() {
    return highposShoulder;
  }
  public double getMidPosElbow() {
    return midposElbow;
  }
  public double getMidPosShoulder() {
    return midposShoulder;
  }
  public double getClosedClawPos() {
    return closedClawPos;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("highposelbow", highposElbow);
    // This method will be called once per scheduler run
  }
}
