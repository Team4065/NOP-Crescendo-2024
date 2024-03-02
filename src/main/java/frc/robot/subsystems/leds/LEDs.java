// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  AddressableLED leds = new AddressableLED(Constants.ledPWMPort);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.ledLength);

  public LEDs() {
    leds.setLength(buffer.getLength());

    for (var i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, 255, 0, 0);
   }
   
   leds.setData(buffer);
   leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
