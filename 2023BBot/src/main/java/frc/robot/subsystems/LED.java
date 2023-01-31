// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  public LED(int port, int length) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static void setColor(AddressableLED led, int r, int g, int b) {
    for(int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
