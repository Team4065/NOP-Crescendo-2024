package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public class GyroIOInputs {
    public boolean isConnected = false;
    public Rotation2d yawPos = new Rotation2d();
    public double yawDouble = 0.0;
    public double yawVelcRadPerSec = 0.0;
    public boolean isReseting = false;
    public double xAccel = 0.0;
    public double yAccel = 0.0;
    public double zAccel = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetGyro() {}
}
