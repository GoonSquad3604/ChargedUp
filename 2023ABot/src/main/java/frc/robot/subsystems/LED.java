// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_length;

  public LED(int port, int num) {
    m_length = num;
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(m_length);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

  }
  /** Sets Color.
   * @param r r value [0-255]
   * @param g g value [0-255]
   * @param b b value [0-255]
   */
  public void setColor(int r, int g, int b) {
    for(int i = 0; i < m_length; i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
