// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();


  public Shooter(ShooterIO io) {
    this.io = io;

  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (!shooterInputs.beamBreak) {
      io.setIntakeVoltage(0);
    }
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }
}
