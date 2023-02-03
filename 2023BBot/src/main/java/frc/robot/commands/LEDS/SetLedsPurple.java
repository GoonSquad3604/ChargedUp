// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class SetLedsPurple extends InstantCommand {
  /** Creates a new SetLedsPurple. */

  private LED m_led;

  public SetLedsPurple(LED led) {
    m_led = led;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("stuff");
    m_led.setColor(255, 0, 255);
  }
}
