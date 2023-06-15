// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.SwerveModuleConstants;


public final class Constants {

    public static final class General{
        public static final double deadband = .1; //drive stick deadband
        public static final String CANIVORE_CANBUS = "drivetrain";
    }

    public static final class swerve{

      public static final double trackWidth = Units.inchesToMeters(28);
      public static final double wheelBase = Units.inchesToMeters(28); 

      public static final int pigeonID = ;
      
      /* Swerve Kinematics 
      * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */

      public static final Translation2d frontLeftLocation = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final Translation2d frontRightLocation = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final Translation2d backLeftLocation = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final Translation2d backRightLocation = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);


       public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          frontLeftLocation,
          frontRightLocation,
          backLeftLocation,
          backRightLocation);

      //Swerve Moudle constants

      public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
      public static final double driveGearRatio = (6.75 / 1.0); 

      public static final boolean driveMotorInvert = true;
      public static final boolean angleMotorInvert = true;
      public static final boolean canCoderInvert = false;

      public static final double wheelCircumference = Units.inchesToMeters(13.0);

      //Angle Motor PID
      public static final double angleKP = 0;
      public static final double angleKI = 0;
      public static final double angleKD = 0;
      public static final double angleKF = 0;

      /* Drive Motor PID Values */

      public static final double driveKP = 0;  
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;

             /* Drive Motor Characterization Values 
       * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      public static final double driveKS = ();
      public static final double driveKV = ();
      public static final double driveKA = ();
      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = ;
      public static final int anglePeakCurrentLimit = ;
      public static final double anglePeakCurrentDuration = ;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveContinuousCurrentLimit = ;
      public static final int drivePeakCurrentLimit = ;
      public static final double drivePeakCurrentDuration = ;
      public static final boolean driveEnableCurrentLimit = true;

      //Swerve Ramping
      public static final double openLoopRamp = ;
      public static final double closedLoopRamp = ;

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 4.968; 
      /** Radians per Second */
      public static final double maxAngularVelocity = 10; 

      /* Neutral Modes */
      public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
      public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class FrontLeft { 
          public static final int driveMotorID = ;
          public static final int angleMotorID = ;
          public static final int canCoderID = ;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees();
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class FrontRight { 
          public static final int driveMotorID = ;
          public static final int angleMotorID = ;
          public static final int canCoderID = ;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees();
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      
      /* Back Left Module - Module 2 */
      public static final class BackLeft { 
          public static final int driveMotorID = ;
          public static final int angleMotorID = ;
          public static final int canCoderID = ;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees();
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class BackRight { 
          public static final int driveMotorID = ;
          public static final int angleMotorID = ;
          public static final int canCoderID = ;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees();
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }


    public static final class ArmConstants {
      public static final int shoulder1ID = ;
      
      public static final int shoulder2ID = ;
      public static final int elbowID = ;
      public static final int clawId = ;

      // Arm lengths
      public static final double bottomArmLength = ; 
      public static final double topArmLength = ; 

      //Arm Origin Length. The Origin of the arm is located how far from the back bumbers and arm.
      public static final double armOriginXOffset = ;
      public static final double armOriginYOffset = ;

      // Gear ratios
      public static final double gearRatio = 1;

      

      // Shoulder PID
      public static final double shoulderUpP = ;
      public static final double shoulderDownP = ;
      public static final double shoulderI = ;
      public static final double shoulderD = ;

      // Elbow PID
      public static final double elbowUpP = ;
      public static final double elbowDownP = ;
      public static final double elbowI = ;
      public static final double elbowD = ;

      // Positions

      // Home Position
      public static final double homeElbow = ;
      public static final double homeShoulder = ;
      
      // Ready to recieve
      public static final double readyElbow = ;


      // Cone
      
      public static final double midConeShoulder = ;
      public static final double midConeElbow = ;

      // Shelf
      public static final double shoulderShelf = ;
      public static final double elbowShelf = ;


      // Cube
     

      // Hybrid
      public static final double hybridShoulder = ;
      public static final double hybridElbow = ;

  }


  public static final class LEDConstants {
    public static final int led1 = 0;
    public static final int led2 = 1;
}

public static final class IntakeConstants {

  // Hinge
  public static final int leftHingeId = ;
  public static final int rightHingeId = ;
  public static final double hingeP = ; 
  public static final double hingeI = 0.00;
  public static final double hingeD = 0;

  public static final double hingeUp = ;
  public static final double hingeShoot = ;
  public static final double hingeDown = ;


  
  public static final int intakeId = ;

  // Intake speeds
  public static final double coneSpeed = ;
  public static final double cubeSpeed = ;
  public static final double vomitSpeed = ;

  public static final int sensorId = ;

  // Shooter speeds
  public static final double topCubeSpeed = ;
  public static final double midCubeSpeed = ;
  public static final double lowCubeSpeed = ;
}

public static final class AutoConstants { 
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final double kSlowMaxSpeedMetersPerSecond = 2.3;
  public static final double kSlowerMaxSpeedMetersPerSecond = 1.5;
  public static final double kSlowerMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kPXController = 0;
  public static final double kPYController = 0.0;
 
  public static final double kPThetaController = 0.0;

  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}


}
