package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
  DCMotorSim driveMotorSim =
      new DCMotorSim(DCMotor.getFalcon500(1), Constants.SwerveConstants.DRIVE_GEAR_RATIO, 0.025);
  DCMotorSim turnMotorSim =
      new DCMotorSim(DCMotor.getFalcon500(1), Constants.SwerveConstants.TURN_GEAR_RATIO, 0.004);

  final Rotation2d turnAbsoluteInitPos = new Rotation2d(Math.random() * 2 * Math.PI);
  double driveAppliedVolts = 0.0;
  double turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotorSim.update(0.02);
    turnMotorSim.update(0.02);

    inputs.drivePositionRad = driveMotorSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveMotorSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnMotorSim.getAngularPositionRad()).plus(turnAbsoluteInitPos);
    inputs.turnPosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnMotorSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotorSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnMotorSim.setInputVoltage(turnAppliedVolts);
  }
}
