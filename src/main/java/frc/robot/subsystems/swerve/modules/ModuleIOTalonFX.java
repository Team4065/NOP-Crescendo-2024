/*
        0 - FRONT LEFT
        1 - FRONT RIGHT
        2 - BACK LEFT
        3 - BACK RIGHT
    */

// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final double DRIVE_GEAR_RATIO = Constants.SwerveConstants.DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO = Constants.SwerveConstants.TURN_GEAR_RATIO;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontLeftDriveID, "*");
        turnTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontLeftTurnID, "*");
        cancoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.frontLeftCanCoderID, "*");
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.frontLeftEncoderOffset; // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightDriveID, "*");
        turnTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightTurnID, "*");
        cancoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.frontRightCanCoderID, "*");
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.frontRightEncoderOffset; // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.backLeftDriveID, "*");
        turnTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.backLeftTurnID, "*");
        cancoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.backLeftCanCoderID, "*");
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.backLeftEncoderOffset; // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.backRightDriveID, "*");
        turnTalon = new TalonFX(Constants.SwerveConstants.ModuleConstants.backRightTurnID, "*");
        cancoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.backRightCanCoderID, "*");
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.backRightEncoderOffset; // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);

    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, drivePosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (driveTalon.getDeviceID() == Constants.SwerveConstants.ModuleConstants.backRightDriveID || driveTalon.getDeviceID() == Constants.SwerveConstants.ModuleConstants.frontRightDriveID) {
      config.Inverted = InvertedValue.Clockwise_Positive;
    } else if (driveTalon.getDeviceID() == Constants.SwerveConstants.ModuleConstants.frontLeftDriveID || driveTalon.getDeviceID() == Constants.SwerveConstants.ModuleConstants.backLeftDriveID) {
      config.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}