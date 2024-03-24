package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  public static AHRS g_gyro = new AHRS(SPI.Port.kMXP);

  public GyroIONavX() {
    g_gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Update gyro values
    inputs.isConnected = g_gyro.isConnected();
    inputs.yawPos = g_gyro.getRotation2d();
    inputs.yawDouble = g_gyro.getYaw();
    inputs.yawVelcRadPerSec = Units.degreesToRadians(g_gyro.getRawGyroZ());
    inputs.isReseting = g_gyro.isCalibrating();
  }

  @Override
  public void resetGyro() {
    g_gyro.reset();
  }
}
