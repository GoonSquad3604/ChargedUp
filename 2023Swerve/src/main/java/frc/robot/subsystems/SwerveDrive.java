// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.swerve.GoonSwerveModule;
import frc.robot.util.swerve.SwerveUtils.Conversions;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */

  private static SwerveDrive _instance;
  public SwerveDriveOdometry swerveDriveOdometry;
  private GoonSwerveModule[] swerveMods; 
  private PigeonIMU m_Pigeon;

  public SwerveDrive() {

    m_Pigeon = new PigeonIMU(Constants.Swerve.pigeonID);
    m_Pigeon.configFactoryDefault();
    zeroGyro();

    swerveMods = new GoonSwerveModule[] {
      new GoonSwerveModule(0, "FrontLeft", Constants.Swerve.FrontLeft.constants),
      new GoonSwerveModule(1, "FrontRight", Constants.Swerve.FrontRight.constants),
      new GoonSwerveModule(2, "BackLeft", Constants.Swerve.BackLeft.constants),
      new GoonSwerveModule(3, "BackRight", Constants.Swerve.BackRight.constants)
    };

    Timer.delay(1);
    resetModulesToAbsolute();
    swerveDriveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,getGyroAngle(), getModulePositions());

  }

  public static final SwerveDrive getInstance() {
    if (_instance == null) {
            _instance = new SwerveDrive();
    }
    return _instance;
  }

  //frontleftRotation is used to changed the center of rotation to the frontleft module instead.
  public void drive(Translation2d translation, double rotation, boolean isOpenLoop, boolean frontleftRotation) {
    //System.out.println("x: " + translation.getX() + " Y: "+ translation.getY() + "ROT: " + rotation);
    SwerveModuleState[] swerveModuleStates = !frontleftRotation ?
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getGyroAngle()
                              )
                            ) :
                            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getGyroAngle()
                              ), Constants.Swerve.frontLeftLocation
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(GoonSwerveModule mod : swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNum], isOpenLoop);
    }
  }    

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
      
      for(GoonSwerveModule mod : swerveMods){
          mod.setDesiredState(desiredStates[mod.moduleNum], false);
      }
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    
    for(GoonSwerveModule mod : swerveMods){
        positions[mod.moduleNum] = mod.getPosition();
        
    }
    return positions;
  }


  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    
    for(GoonSwerveModule mod : swerveMods){
      states[mod.moduleNum] = mod.getState();
        
    }
    return states;
  }

  public void zeroGyro(){
    m_Pigeon.setYaw(0);
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_Pigeon.getYaw());
  }

  public void resetOdometry(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
  }

  public void resetModulesToAbsolute(){
    for(GoonSwerveModule mod : swerveMods){
        mod.resetToAbsolute();
    }
}
  
  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    for(GoonSwerveModule mod : swerveMods){
      SmartDashboard.putNumber(mod.name + " Can Coder Angle", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(mod.name + " talon angle", mod.getAngle().getDegrees());
      SmartDashboard.putNumber(mod.name + " Encoder clicks", mod.getEncoderClicks());
    }
    swerveDriveOdometry.update(getGyroAngle(), getModulePositions());
    printclicks();

    SmartDashboard.putNumber("gyro angle", getGyroAngle().getDegrees());
  }

  public void setAll25() {
    for(GoonSwerveModule mod : swerveMods){
      if(mod.moduleNum == 0)
        mod.set25();
    }
  }

  public void printclicks(){
    double clicks = swerveMods[0].getEncoderClicks();
    SmartDashboard.putNumber("clicks before", clicks);
    double degrees = Conversions.falconToDegrees(clicks, Constants.Swerve.angleGearRatio);
    SmartDashboard.putNumber("degrees", degrees);
    double newclicks = Conversions.degreesToFalcon(degrees, Constants.Swerve.angleGearRatio);
    SmartDashboard.putNumber("clicks after", newclicks);


  }
}
