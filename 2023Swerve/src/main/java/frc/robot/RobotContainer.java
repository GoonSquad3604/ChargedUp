// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autons.ExampleAuton;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Declare controllers
  private XboxController driver = new XboxController(0);


  //Declare Subsystems
  private SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    s_SwerveDrive.setDefaultCommand(
            new SwerveDefaultDrive(() -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX()));

    configureBindings();
  }

  
  private void configureBindings() {
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);

    driverY.onTrue(new InstantCommand(() -> s_SwerveDrive.zeroGyro()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    ExampleAuton exampleAuton = new ExampleAuton();
    s_SwerveDrive.resetOdometry(exampleAuton.getInitialPose());

    return exampleAuton;
  }
}
