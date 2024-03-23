// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private GenericEntry beamBreakEntry = Constants.dataTab.add("BEAMBREAK", true).getEntry();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    Pose3d[] elevatorPoses = RobotContainer.m_elevator.getPoses3d();
    Pose3d[] cameraPoses = RobotContainer.m_vision.cameraPoses();

    Logger.recordOutput("Mechanism3d", 
      elevatorPoses[0], 
      elevatorPoses[1], 
      cameraPoses[0], 
      cameraPoses[1],
      cameraPoses[2],
      cameraPoses[3]
    );

    beamBreakEntry.setBoolean(shooterInputs.beamBreak);    
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  public double getShooterVelc() {
    return shooterInputs.topRollerVelocityRadPerSec;
  }

  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public boolean getBeamBreak() {
    return shooterInputs.beamBreak;
  }

  public void goBackWards() {
    
  }
}
