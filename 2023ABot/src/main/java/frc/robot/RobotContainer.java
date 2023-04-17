// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.locks.Condition;

import org.ejml.masks.FMaskPrimitive;

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
import frc.robot.commands.StopAll;
import frc.robot.commands.LEDS.SetLedsPurple;
import frc.robot.commands.LEDS.SetLedsWhite;
import frc.robot.commands.LEDS.SetLedsYellow;
import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.commands.arm.ArmHigh;
import frc.robot.commands.arm.ArmHighCube;
import frc.robot.commands.arm.ArmLow;
import frc.robot.commands.arm.ArmMedium;
import frc.robot.commands.arm.ArmMediumCube;
import frc.robot.commands.arm.ArmShelf;
import frc.robot.commands.arm.HomePosition;
import frc.robot.commands.arm.HomePositionCone;
import frc.robot.commands.arm.ReadyToRecieve;
import frc.robot.commands.autons.AutonTest;
import frc.robot.commands.autons.ForwardAndBack;
import frc.robot.commands.autons.OnePieceMid;
import frc.robot.commands.autons.OnePieceMidBalance;
import frc.robot.commands.autons.OnePieceMidBalanceCube;
import frc.robot.commands.autons.OnePieceOfflane;
import frc.robot.commands.autons.TestAuton;
import frc.robot.commands.autons.ThreeCubeAuton;
import frc.robot.commands.autons.ThreePieceAuton;
import frc.robot.commands.autons.TwoPieceFreelane;
import frc.robot.commands.autons.TwoPieceFreelaneBalance;
import frc.robot.commands.drive.Aim;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.CenterPole;
import frc.robot.commands.drive.DefaultAngle;
import frc.robot.commands.drive.FastAutoBalance;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.intake.ToggleHinge;
import frc.robot.commands.intake.ToggleHingeDown;
import frc.robot.commands.states.SetConeMode;
import frc.robot.commands.states.SetCubeMode;
import frc.robot.commands.utils.Wait;
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
 

  //Declare Certain Buttons NICEEEEE
  private JoystickButton driverLeftBumber = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
  
  //Declare Subsystems
  private SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();
  private Intake s_Intake = Intake.getInstance();
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
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Mid Single Cone", new OnePieceMid(s_LED, s_Intake));
    m_chooser.addOption("Mid Cone Balance", new OnePieceMidBalance(s_LED, s_Intake));
    m_chooser.addOption("GODTON", new TwoPieceFreelaneBalance(s_LED, s_Intake));
    m_chooser.addOption("Three Piece Free Lane", new ThreePieceAuton(s_LED, s_Intake));
    m_chooser.addOption("Off Lane Single Cone Back Up", new OnePieceOfflane(s_LED, s_Intake));
    
    SmartDashboard.putData("chooser", m_chooser);

    s_LED.setColor(255, 255, 255);

    configureBindings();

  }

  private void configureBindings() {

    // Driver
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    JoystickButton driverStop = new JoystickButton(driver, XboxController.Button.kBack.value);

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

    Trigger intakeSensorTrigger = new Trigger(s_Intake::getIntakeSensor);

    Trigger driverRightTrigger = new Trigger(()->driver.getRightTriggerAxis() > 0.8);

    // MANUAL STUFF

    // Claw
    // operatorY.onTrue(new InstantCommand(() -> s_Arm.moveClaw(0.2)));
    // operatorY.onFalse(new InstantCommand(() -> s_Arm.moveClaw(0)));
    // operatorX.onTrue(new InstantCommand(() -> s_Arm.moveClaw(-0.2)));
    // operatorX.onFalse(new InstantCommand(() -> s_Arm.moveClaw(0)));
    operatorY.onTrue(new InstantCommand(() -> s_Intake.setHinge(-.2, 0)));
    operatorY.onFalse(new InstantCommand(() -> s_Intake.setHinge(0,0)));
    operatorX.onTrue(new InstantCommand(() -> s_Intake.setHinge(.2, 0)));
    operatorX.onFalse(new InstantCommand(() -> s_Intake.setHinge(0,0)));
    // operator2.onTrue(new ToggleHinge());

    //driverX.onTrue(new FastAutoBalance());

    // Hinge
    // operator3.onTrue(new InstantCommand(() -> s_Intake.setHinge(0.2, 0.2)));
    // operator3.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
    // operator4.onTrue(new InstantCommand(() -> s_Intake.setHinge(-0.15, -0.15)));
    // operator4.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));

    // Intake
    operator1.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    operator1.and(intakeSensorTrigger).onTrue(new InstantCommand(() -> s_Intake.stopIntake()));

    operator1.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator2.onTrue(new InstantCommand(() -> s_Intake.hingeTo(Constants.IntakeConstants.hingeDown)));
    operator3.onTrue(new InstantCommand(() -> s_Intake.vomit()));
    operator3.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    operator7.onTrue(new SetCubeMode(s_LED));
    operator8.onTrue(new SetConeMode(s_LED));

    // HINGE ZERO
    driverB.onTrue(new InstantCommand(() -> s_Intake.setHinge(-.2, -0.15)));
    driverB.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
   


    // SMART STUFF

    // HOLY MYOJ OAJGJ A J STOOOP
    driverStart.and(driverStop).onTrue(new StopAll());

    // Drive Train
    

    // Arm
    operator9.and(coneTrigger).onTrue(new HomePositionCone());
    operator10.and(coneTrigger).onTrue(new ArmHigh());
    operator11.and(coneTrigger).onTrue(new ArmMedium());
    operator12.and(coneTrigger).onTrue(new ArmLow());

    driverRightTrigger.and(coneTrigger).onTrue(new InstantCommand(()-> s_Shoulder.shoulderTo(s_Shoulder.getShoulderClicks()+5)));
    driverA.and(coneTrigger).onTrue(new InstantCommand(()-> s_Shoulder.shoulderTo(s_Shoulder.getShoulderClicks()-5)));

    // Claw
    operator4.and(coneTrigger).onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> s_Arm.clawTo(s_StateController.getClosedClawPos())),
      new Wait(0.5),
      new InstantCommand(() -> s_LED.setColor(255, 255, 0)),
      new Wait(0.5),
      new InstantCommand(() -> s_Arm.stopClaw())
    ));

    //operator4.and(coneTrigger).onTrue(new InstantCommand(() -> s_LED.setColor(255, 255, 0)));

    operator5.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> s_Arm.clawTo(Constants.ArmConstants.startingPos)),
      new Wait(1.0),
      new InstantCommand(() -> s_Arm.stopClaw())
    ));

    operatorA.onTrue(new InstantCommand(() -> s_Arm.clawTo(Constants.ArmConstants.autonReady)));
 

    operator6.and(coneTrigger).onTrue(new ArmShelf());
    operator6.and(coneTrigger).onTrue(new InstantCommand(() -> s_LED.setColor(255, 0, 0)));
    operator6.and(cubeTrigger).onTrue(new InstantCommand(() -> s_Intake.hingeTo(Constants.IntakeConstants.hingeShoot)));
  
    driverY.onTrue(new InstantCommand(() -> s_SwerveDrive.zeroGyro()));

    // PURPLE CANNON!!!
    operator9.and(cubeTrigger).onTrue(new InstantCommand(() -> s_Intake.hingeTo(Constants.IntakeConstants.hingeUp)));
    operator10.and(cubeTrigger).onTrue(new InstantCommand(() -> s_Intake.runIntake(Constants.IntakeConstants.topCubeSpeed)));
    operator11.and(cubeTrigger).onTrue(new InstantCommand(() -> s_Intake.runIntake(Constants.IntakeConstants.midCubeSpeed)));
    operator12.and(cubeTrigger).onTrue(new InstantCommand(() -> s_Intake.runIntake(Constants.IntakeConstants.lowCubeSpeed)));
    
    operator10.and(cubeTrigger).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator11.and(cubeTrigger).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator12.and(cubeTrigger).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //GoonAutonCommand auton = new TestAuton(s_LED, s_Intake);
    GoonAutonCommand auton = m_chooser.getSelected();
    // GoonAutonCommand auton = new AutonTest(s_LED, s_Intake);

     //GoonAutonCommand auton = new OnePieceMidBalance(s_LED, s_Intake);
    //GoonAutonCommand auton = new TwoPieceFreelaneBalance(s_LED, s_Intake);
    //GoonAutonCommand auton = new ThreePieceAuton(s_LED, s_Intake);
    
    s_SwerveDrive.resetOdometry(auton.getInitialPose());
    return auton;
  }

}
