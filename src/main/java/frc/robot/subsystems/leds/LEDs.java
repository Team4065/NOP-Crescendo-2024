// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  AddressableLED leds = new AddressableLED(Constants.ledPWMPort);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.ledLength);
  String state = "idle";

  private double m_interval = 0.5;
  private boolean on = true;
  private double lastChange;

  public LEDs() {
    leds.start();
  }

  public void oneColor(int R, int G, int B) {
    leds.setLength(buffer.getLength());

    for (var i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, R, G, B);
    }
   
    leds.setData(buffer);
  }

  public void strobing() {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - lastChange > m_interval) {
      on = !on;
    }

    if (on) {
      oneColor(255, 255, 255);
    } else {
      oneColor(0, 0, 0);
    }
  }



  public void setState(String state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    switch (state) {
      case "idle":
        if (RobotContainer.m_shooter.getBeamBreak() == true) {
          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            oneColor(255, 0, 0);
          } else {
            oneColor(0, 0, 255);
          }
        } else {
          oneColor(0, 255, 0); // orange
        }
        
        break;
      
      case "green":
        oneColor(0, 255, 0);

        break;
      
      case "nothing":
        oneColor(0, 0, 0);

        break;

      case "rainbow":
        strobing();
      
        break;
      case "amplify":
        oneColor(215, 3, 252);

        break;
      
      default:
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          oneColor(255, 0, 0);
        } else {
          oneColor(0, 0, 255);
        }
        
        break;
    }
    leds.start();
  }
  
}
