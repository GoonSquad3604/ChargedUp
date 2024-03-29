// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.Proxy.Type;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private static Vision _instance; 

  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private boolean hasTarget = false;

  private double tx;
  private double ty;
  private double ta;

  private final double cameraHeight = 0.17;
  private final double cameraAngle = Units.degreesToRadians(20);
  private final double targetLowerHeight = 0.59;
  private double distance;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("photonvision");

  }

  public static Vision getInstance(){
    if(_instance == null){
      _instance = new Vision();
    }

    return _instance;

  }
  /** @return Returns the boolean if the vision has the target or not. */
  public boolean getHasTarget(){
    return hasTarget;
  }
  /** @return Returns the x position of the target on the vision. */
  public double getTx(){
    return tx;
  }
  /** @return Returns the y position of the target on the vision */
  public double getTy(){
    return ty;
  }
  /** @return The angle of the target. */
  public double getTa(){
    return ta;
  }
  /** Approximates the distance from the bot to the target using a line of best fit and prints out the distance on smart dashboard. */
  public void getDistance() {

    distance = Math.pow(3.3, -(ty/16.5)+0.27);
    
    SmartDashboard.putNumber("Distance", distance);
  }

  @Override
  public void periodic() {
    
    result = camera.getLatestResult();
    hasTarget = result.hasTargets();

    if(hasTarget){
      tx = result.getBestTarget().getYaw();
      ty = result.getBestTarget().getPitch();
      ta = result.getBestTarget().getArea();
    }

    getDistance();
    SmartDashboard.putBoolean("hastarget", hasTarget);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);
  }
}
