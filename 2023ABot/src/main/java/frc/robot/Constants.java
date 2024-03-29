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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class General{
        public static final double deadband = .1; //drive stick deadband
        public static final String CANIVORE_CANBUS = "drivetrain";
    }

    public static final class Swerve{

        // /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23); 
        public static final double wheelBase = Units.inchesToMeters(23); 

        public static final int pigeonID = 25;
        
        /* Swerve Kinematics 
        * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */

        //meaning of life, the universe and everything
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
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        //Swerve Ramping
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.968; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft { 
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.54);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(78.750000);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeft { 
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(20.039063);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRight { 
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(71.367188);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class ArmConstants {
        public static final int shoulder1ID = 15;
        
        public static final int shoulder2ID = 4;
        public static final int elbowID = 5;
        public static final int clawId = 14;

        // Arm lengths
        public static final double bottomArmLength = 5; 
        public static final double topArmLength = 5; 

        //Arm Origin Length. The Origin of the arm is located how far from the back bumbers and arm.
        public static final double armOriginXOffset = .3326;
        public static final double armOriginYOffset = 1.07;

        // Gear ratios
        public static final double gearRatio = 1;

        // Claw PID
        public static final double clawP = 24;
        public static final double clawI = 0;
        public static final double clawD = 3;
        public static final double clawFF = 0.1;

        // Shoulder PIDb
        public static final double shoulderUpP = 7.0;
        public static final double shoulderDownP = 7.0;
        public static final double shoulderI = 0;
        public static final double shoulderD = 0.00;

        // Elbow PID
        public static final double elbowUpP = 4.0;
        public static final double elbowDownP = 4.0;
        public static final double elbowI = 0;
        public static final double elbowD = 0;

        // Positions

        // Home Position
        public static final double homeElbow = 312.9;
        public static final double homeShoulder = 108.14;
        
        // Ready to recieve
        public static final double readyElbow = 166.16;


        // Cone
        public static final double highConeShoulder = 205.73;
        public static final double highConeElbow = 163.3;

        public static final double midConeShoulder = 193.3;
        public static final double midConeElbow = 177;

        // Shelf
        public static final double shoulderShelf = 113.04;
        public static final double elbowShelf = 313.58;


        // Cube
        public static final double highCubeShoulder = 202.24;
        public static final double highCubeElbow = 266.927369;

        public static final double midCubeShoulder = 182.93;
        public static final double midCubeElbow = 288.625817;

        // Hybrid
        public static final double hybridShoulder = 135.2;
        public static final double hybridElbow = 256;

        // Claw
        public static final double closedCone = 0.305;
        public static final double closedCube = 0.315;
        public static final double startingPos = 0.41;
        public static final double autonReady = 0.359;
        public static final double shelfPos = 0.41;




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

        public static final double hingeUp = 0.51;
        public static final double hingeShoot = 0.572668;
        public static final double hingeDown = 0.803; //0.808;


        // Spaghetti
        public static final int intakeId = 6;

        // Intake speeds
        public static final double coneSpeed = 0.75;
        public static final double cubeSpeed = 0.35;
        public static final double vomitSpeed = -1.0;

        public static final int sensorId = 9;

        // Shooter speeds
        public static final double topCubeSpeed = -1.0;
        public static final double midCubeSpeed = -0.55;
        public static final double lowCubeSpeed = -0.4;


    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kSlowMaxSpeedMetersPerSecond = 2.3;
        public static final double kSlowerMaxSpeedMetersPerSecond = 1.5;
        public static final double kSlowerMaxAccelerationMetersPerSecondSquared = 3;
    
        public static final double kPXController = 8.0;
        public static final double kPYController = 0.0;
       
        public static final double kPThetaController = 0.0;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
