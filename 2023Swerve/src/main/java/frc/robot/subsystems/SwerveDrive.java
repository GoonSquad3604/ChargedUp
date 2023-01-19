// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.swerve.GoonSwerveModule;

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

    swerveDriveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,getGyroAngle(), getModulePositions());

    swerveMods = new GoonSwerveModule[] {
      new GoonSwerveModule(0, "FrontLeft", Constants.Swerve.FrontLeft.constants),
      new GoonSwerveModule(1, "FrontRight", Constants.Swerve.FrontRight.constants),
      new GoonSwerveModule(2, "BackLeft", Constants.Swerve.BackLeft.constants),
      new GoonSwerveModule(3, "BackRight", Constants.Swerve.BackRight.constants)
    };

  }

  public static final SwerveDrive getInstance() {
    if (_instance == null) {
            _instance = new SwerveDrive();
    }
    return _instance;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getGyroAngle()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
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
  
  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // for(GoonSwerveModule mod : swerveMods){
    //   SmartDashboard.putNumber(mod.name + " Angle", 0)
    // }
  }
}
