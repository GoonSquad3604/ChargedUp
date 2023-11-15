// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private XboxController driver = new XboxController(0);
  private XboxController operator = new XboxController(3);
  Autos auton = new Autos();

  private static Shooter s_Shooter = Shooter.getInstance();
  private DriveTrain s_DriveTrain = DriveTrain.getInstance();
  private static Indexer s_Indexer = Indexer.getInstance();
  private static Intake mainIntake = Intake.getInstance();

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    s_DriveTrain.setDefaultCommand(new DefaultDrive(() -> driver.getLeftY(), () -> driver.getRightX()));
    configureBindings();
  }

  /**%
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    JoystickButton operatorJoystickForward = new JoystickButton(operator, XboxController.Button.kY.value);
    JoystickButton operatorJoystickBack = new JoystickButton(operator, XboxController.Button.kX.value);
    driverB.onTrue(new InstantCommand(()-> s_Shooter.setPower(1)));
    driverB.onFalse(new InstantCommand(()-> s_Shooter.setPower(0)));

    //driverA.onTrue(new InstantCommand(()-> s_Shooter.setTurretTo(.3)));
    //driverA.onFalse(new InstantCommand(()->s_Shooter.setTurretTo(0)));
    //driverY.onTrue(new InstantCommand(()->s_Shooter.turretReverse(.3)));
    //driverY.onFalse(new InstantCommand(()-> s_Shooter.setTurretTo(0)));
    driverLB.onTrue(new InstantCommand(()->s_Indexer.moveIndex1(1)));
    driverLB.onFalse(new InstantCommand(()->s_Indexer.moveIndex1(0)));
    driverRB.onTrue(new InstantCommand(()->s_Indexer.moveIndex2(1)));
    driverRB.onFalse(new InstantCommand(()->s_Indexer.moveIndex2(0)));
    operatorJoystickForward.onTrue(new InstantCommand(()->mainIntake.runIntake(0.6)));
    operatorJoystickForward.onFalse(new InstantCommand(()->mainIntake.stopIntake(0)));
    operatorJoystickBack.onTrue(new InstantCommand(()->mainIntake.runIntake(-0.6)));
    operatorJoystickBack.onFalse(new InstantCommand(()->mainIntake.stopIntake(0)));
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//    public Command getAutonomousCommand() {
//    }
// }
}