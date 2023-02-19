// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDS.SetLedsPurple;
import frc.robot.commands.LEDS.SetLedsWhite;
import frc.robot.commands.LEDS.SetLedsYellow;
import frc.robot.commands.arm.ArmDefaultCommand;
import frc.robot.commands.autons.TestAuton;
import frc.robot.commands.drive.Aim;
import frc.robot.commands.drive.CenterPole;
import frc.robot.commands.drive.DefaultAngle;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.stick.StickDefaultCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Declare controllers
  private XboxController driver = new XboxController(0);
  private XboxController operator = new XboxController(1);
  TestAuton auton = new TestAuton();


  
  //Declare Certain Buttons
  private JoystickButton driverLeftBumber = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  

  //Declare Subsystems
  private SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();
  //private Stick s_Stick = Stick.getInstance();
  private Vision s_Vision = new Vision();

  //private Arm s_Arm = Arm.getInstance();
  //private LED s_LED = new LED(Constants.LEDConstants.led1, 60);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //double speedBoost = 1.0;


    //Set Defualt Commands
    s_SwerveDrive.setDefaultCommand(
            new SwerveDefaultDrive(() -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), driverLeftBumber, driverRightBumper, () -> driver.getLeftTriggerAxis()));

    //s_Stick.setDefaultCommand(new StickDefaultCommand(() -> operator.getLeftY()));

    //s_Arm.setDefaultCommand(new ArmDefaultCommand(() -> operator.getLeftY(), () -> operator.getRightY()));

    configureBindings();

    SmartDashboard.putString("auton pose", auton.getInitialPose().toString());
    //s_SwerveDrive.resetOdometry(auton.getInitialPose());
    // s_SwerveDrive.resetModulesToAbsolute();
  }


  
  private void configureBindings() {
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);



    // operatorY.onTrue(new SetLedsPurple(s_LED));
    // operatorX.onTrue(new SetLedsWhite(s_LED));
    // operatorA.onTrue(new SetLedsYellow(s_LED));

    driverY.onTrue(new InstantCommand(() -> s_SwerveDrive.zeroGyro()));
    driverB.onTrue(new DefaultAngle(s_SwerveDrive, driver));
    driverA.onTrue(new CenterPole(s_Vision, s_SwerveDrive, driver));
    //driverLeftBumper.whileTrue(() -> (speedBoost = 0.5;));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TestAuton auton = new TestAuton();
    s_SwerveDrive.resetOdometry(auton.getInitialPose());
    return auton;
  }

}
