// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
  AddressableLED m_led;
  AddressableLEDBuffer m_LedBuffer;

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    m_led = new AddressableLED(LedConstants.PWM_LED_PORT);
    m_LedBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);

    m_led.setLength(m_LedBuffer.getLength());
    m_led.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(LEDPattern pattern) {
    pattern.applyTo(m_LedBuffer);
    m_led.setData(m_LedBuffer);
  }

  public Color getColor(int index) {
    return m_LedBuffer.getLED(index);
  }

  public void turnOff() {
    setColor(LEDPattern.solid(Color.kBlack));
  }

  public Command setPattern(LEDPattern pattern) {
    return this.runOnce(() -> setColor(pattern));
  }

}
