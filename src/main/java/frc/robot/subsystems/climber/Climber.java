// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.right.RightRatchet;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);

    // if (RobotContainer.B5.getAsBoolean() == false && RobotContainer.B7.getAsBoolean() == false) {
    //   setRightSpeed(0);
    //   setRightRatchet(false);
    // }

    // if (RobotContainer.B4.getAsBoolean() == false && RobotContainer.B6.getAsBoolean() == false) {
    //   setLeftSpeed(0);
    //   setLeftRatchet(false);
    // }
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setRatchet(boolean state) {
    io.setRatchet(state);
  }

  public void setRightSpeed(double speed) {
    io.setRightSpeed(speed);
  }

  public void setLeftSpeed(double speed) {
    io.setLeftSpeed(speed);
  }

  public void setRightRatchet(boolean state) {
    io.setRightRatchet(state);
  }

  public void setLeftRatchet(boolean leftState) {
    io.setLeftRatchet(leftState);
  }
}
