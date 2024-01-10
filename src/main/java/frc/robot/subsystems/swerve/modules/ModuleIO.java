package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double [] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveVoltage(double voltage) {}

  public default void setTurnVoltage(double voltage) {}

  public default void setDriveBrakeMode(boolean enable) {}

  public default void setTurnBrakeMode(boolean enable) {}
}
