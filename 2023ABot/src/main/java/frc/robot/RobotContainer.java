// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDS.SetLedsPurple;
import frc.robot.commands.LEDS.SetLedsWhite;
import frc.robot.commands.LEDS.SetLedsYellow;
import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.arm.ArmHighCube;
import frc.robot.commands.arm.ArmLow;
import frc.robot.commands.arm.ArmMedium;
import frc.robot.commands.arm.ArmMediumCube;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.autons.OnePieceMid;
import frc.robot.commands.autons.OnePieceOfflane;
import frc.robot.commands.autons.TestAuton;
import frc.robot.commands.autons.TwoPieceFreelane;
import frc.robot.commands.drive.Aim;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.CenterPole;
import frc.robot.commands.drive.DefaultAngle;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.intake.ToggleHinge;
import frc.robot.commands.intake.ToggleHingeDown;
import frc.robot.commands.states.SetConeMode;
import frc.robot.commands.states.SetCubeMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.util.auton.GoonAutonCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Declare controllers
  private XboxController driver = new XboxController(0);
  private XboxController operatorController = new XboxController(1);
  private Joystick operatorJoystick = new Joystick(2);
 

  //Declare Certain Buttons
  private JoystickButton driverLeftBumber = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
  
  //Declare Subsystems
  private SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();
  private Intake s_Intake = new Intake();
  private StateController s_StateController = StateController.getInstance();
  
  //private Vision s_Vision = new Vision();

  private Arm s_Arm = Arm.getInstance();
  private Shoulder s_Shoulder = Shoulder.getInstance();
  private LED s_LED = new LED(Constants.LEDConstants.led1, 24);


  //Declare Autons and chooser
  SendableChooser<GoonAutonCommand> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Set Defualt Commands%
    s_SwerveDrive.setDefaultCommand(
            new SwerveDefaultDrive(() -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), driverLeftBumber, driverRightBumper, () -> driver.getLeftTriggerAxis()));

    s_Arm.setDefaultCommand(new ArmDefaultCommand(() -> operatorController.getLeftY(), () -> operatorController.getRightY(), operatorLeftBumper));

    //Build Auton Chooser
    m_chooser = new SendableChooser<GoonAutonCommand>();
    m_chooser.setDefaultOption("Mid Single Cone", new OnePieceMid(s_LED, s_Intake));
    m_chooser.addOption("Two Piece Free Lane", new TwoPieceFreelane(s_LED, s_Intake));
    m_chooser.addOption("Off Lane Single Piece Back Up", new OnePieceOfflane(s_LED, s_Intake));
    SmartDashboard.putData(m_chooser);

    s_LED.setColor(255, 255, 255);

    configureBindings();

  }

  private void configureBindings() {

    // Driver
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);

    // Operator
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    
    JoystickButton operator1 = new JoystickButton(operatorJoystick, 1);
    JoystickButton operator2 = new JoystickButton(operatorJoystick, 2);
    JoystickButton operator3 = new JoystickButton(operatorJoystick, 3);
    JoystickButton operator4 = new JoystickButton(operatorJoystick, 4);
    JoystickButton operator5 = new JoystickButton(operatorJoystick, 5);
    JoystickButton operator6 = new JoystickButton(operatorJoystick, 6);
    JoystickButton operator7 = new JoystickButton(operatorJoystick, 7);
    JoystickButton operator8 = new JoystickButton(operatorJoystick, 8);
    JoystickButton operator9 = new JoystickButton(operatorJoystick, 9);
    JoystickButton operator10 = new JoystickButton(operatorJoystick, 10);
    JoystickButton operator11 = new JoystickButton(operatorJoystick, 11);
    JoystickButton operator12 = new JoystickButton(operatorJoystick, 12);

    //custom triggers
    Trigger toggleTrigger = new Trigger(s_Intake::getToggle);
    Trigger notToggleTrigger = new Trigger(s_Intake::getNotToggle);

    Trigger coneTrigger = new Trigger(s_StateController::isConeMode);
    Trigger cubeTrigger = new Trigger(s_StateController::isCubeMode);

    // MANUAL STUFF

    // Claw
    operatorY.onTrue(new InstantCommand(() -> s_Arm.moveClaw(0.2)));
    operatorY.onFalse(new InstantCommand(() -> s_Arm.moveClaw(0)));
    operatorX.onTrue(new InstantCommand(() -> s_Arm.moveClaw(-0.2)));
    operatorX.onFalse(new InstantCommand(() -> s_Arm.moveClaw(0)));
    // operator2.onTrue(new ToggleHinge());

    // Hinge
    // operator3.onTrue(new InstantCommand(() -> s_Intake.setHinge(0.2, 0.2)));
    // operator3.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
    // operator4.onTrue(new InstantCommand(() -> s_Intake.setHinge(-0.15, -0.15)));
    // operator4.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));

    // Intake
    operator1.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    operator1.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator3.onTrue(new InstantCommand(() -> s_Intake.vomit()));
    operator3.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    operator7.onTrue(new SetCubeMode(s_LED));
    operator8.onTrue(new SetConeMode(s_LED));

    // HINGE ZERO
    driverB.onTrue(new InstantCommand(() -> s_Intake.setHinge(-0.15, -0.15)));
    driverB.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
    driverB.onFalse(new InstantCommand(() -> s_Intake.zeroHinge()));


    // SMART STUFF

    // Drive Train
    //driverA.onTrue(new AutoBalance(s_SwerveDrive));

    // Arm
    operator9.onTrue(new HomePosition());
    //operator10.onTrue(new ArmHigh(s_StateController.getHighPosShoulder(),s_StateController.getHighPosElbow()));
    operator10.and(coneTrigger).onTrue(new ArmHigh());
    operator10.and(cubeTrigger).onTrue(new ArmHighCube());
    
    operator11.and(coneTrigger).onTrue(new ArmMedium());
    operator11.and(cubeTrigger).onTrue(new ArmMediumCube());

    operator12.onTrue(new ArmLow());

    // Claw
    operator4.onTrue(new InstantCommand(() -> s_Arm.clawTo(s_StateController.getClosedClawPos())));
    operator5.onTrue(new InstantCommand(() -> s_Arm.clawTo(0)));
    operator2.and(toggleTrigger).onTrue(new ToggleHinge(s_Intake)); 
    operator2.and(notToggleTrigger).onTrue(new ToggleHingeDown(s_Intake)); 
    operatorA.onTrue(new InstantCommand(() -> s_Arm.clawTo(Constants.ArmConstants.startingPos)));
 

    //operator6.onTrue(new ReadyToRecieve());
  
    driverY.onTrue(new InstantCommand(() -> s_SwerveDrive.zeroGyro()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    GoonAutonCommand auton = m_chooser.getSelected();
    
    s_SwerveDrive.resetOdometry(auton.getInitialPose());
    return auton;
  }

}
