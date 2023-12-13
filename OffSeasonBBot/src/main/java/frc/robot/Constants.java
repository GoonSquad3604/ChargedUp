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

    public static final class Swerve{

      public static final double trackWidth = Units.inchesToMeters(28);
      public static final double wheelBase = Units.inchesToMeters(28); 

      public static final int pigeonID = 25;
      
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
      public static final double angleKP = 0.3;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKF = 0.0;

      /* Drive Motor PID Values */

      public static final double driveKP = 0.05171;  
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;

             /* Drive Motor Characterization Values 
       * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      public static final double driveKS = (0.44576 / 12);
        public static final double driveKV = (2.9137 / 12);
        public static final double driveKA = (0.7054 / 12);
      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = 25;
      public static final int anglePeakCurrentLimit = 40;
      public static final double anglePeakCurrentDuration = .1 ;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveContinuousCurrentLimit = 35;
      public static final int drivePeakCurrentLimit = 60;
      public static final double drivePeakCurrentDuration =.1 ;
      public static final boolean driveEnableCurrentLimit = true;

      //Swerve Ramping
      public static final double openLoopRamp = .25;
      public static final double closedLoopRamp = 0;

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 5.1; 
      /** Radians per Second */
      public static final double maxAngularVelocity = 10; 

      /* Neutral Modes */
      public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
      public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class FrontLeft { 
          public static final int driveMotorID = 18;
          public static final int angleMotorID = 16;
          public static final int canCoderID = 20;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(284.24);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class FrontRight { 
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 21;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(310.78);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      
      /* Back Left Module - Module 2 */
      public static final class BackLeft { 
          public static final int driveMotorID = 19;
          public static final int angleMotorID = 17;
          public static final int canCoderID = 22;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(194.59);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class BackRight { 
          public static final int driveMotorID = 40;
          public static final int angleMotorID = 2;
          public static final int canCoderID = 23;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(77.61);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }


    public static final class ArmConstants {
      public static final int shoulder1ID = 15;
      
      public static final int shoulder2ID = 5;
      public static final int elbowID = 14;
      public static final int rollerID = 8;

      // Arm lengths
      public static final double bottomArmLength = 4; 
      public static final double topArmLength = 4; 

      //Arm Origin Length. The Origin of the arm is located how far from the back bumbers and arm.
      public static final double armOriginXOffset = 4;
      public static final double armOriginYOffset = 4;

      // Gear ratios
      public static final double gearRatio = 1;

      

      // Shoulder PID
      public static final double shoulderUpP = 7;
      public static final double shoulderDownP = 7;
      public static final double shoulderI = 0;
      public static final double shoulderD = 0;

      // Elbow PID
      public static final double elbowUpP = 4 ;
      public static final double elbowDownP = 4;
      public static final double elbowI = 0;
      public static final double elbowD = 0;

      // Positions

      // Home Position
      public static final double homeElbow = 114.14;
      public static final double homeShoulder = 244.07;
      
      // Ready to recieve
      public static final double readyElbow = 0;


      // Cone
      
      public static final double midConeShoulder = 207.19;
      public static final double midConeElbow = 204.07;

      // Shelf
      public static final double shoulderShelf = 154.89;
      public static final double elbowShelf = 278.145;


      public static final double rollerCurrentLimit = 1;


      // Cube
     

      // Hybrid
      public static final double hybridShoulder = 0;
      public static final double hybridElbow = 0;

  }


  public static final class LEDConstants {
    public static final int led1 = 0;
    public static final int led2 = 1;
}

public static final class IntakeConstants {

  // Hinge
  public static final int leftHingeId = 13;
  public static final int rightHingeId = 7;
  public static final double hingeP = 2; 
  public static final double hingeI = 0.00;
  public static final double hingeD = 0;

  public static final double hingeUp = 0.358;
  public static final double hingeShoot = 0.402;
  public static final double hingeDown = 0.636;


  
  public static final int intakeId = 6;

  // Intake speeds
  public static final double coneSpeed = .75;
  public static final double cubeSpeed = .35;
  public static final double vomitSpeed = -1;

  public static final int sensorId = 9;

  // Shooter speeds
  public static final double topCubeSpeed = -.85;
  public static final double midCubeSpeed = -.55;
  public static final double lowCubeSpeed = -.4;
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

