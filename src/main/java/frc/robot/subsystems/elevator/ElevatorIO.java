package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public Rotation2d rightTiltPositionRad = new Rotation2d();
        public double rightTiltVelocityRadPerSec = 0.0;
        public double rightTiltAppliedVolts = 0.0;
        public double rightTiltCurrentAmps = 0.0;

        public Rotation2d leftTiltPositionRad = new Rotation2d();
        public double leftTiltVelocityRadPerSec = 0.0;
        public double leftTiltAppliedVolts = 0.0;
        public double leftTiltCurrentAmps = 0.0;

        public boolean tiltReached = false;

        public Rotation2d absoluteTiltPositionRad = new Rotation2d();
        public double absoluteDeg = 0.0;
        public double absoluteVelc = 0.0;

        public double elevatorEncoder = 0;
        public Rotation2d elevatorPositionRad = new Rotation2d();
        public double elevatorVelocityRadPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;
        public boolean elevatorLimitReached = false;
        public double elevatorLinearVelocity = 0.0;

        public boolean neturalModeButton = false;
        public boolean isBrakeMode = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setTiltVoltage(double volts) {}

    public default void setElevatorVoltage (double volts) {}

    public default void setBrakeMode(boolean state) {}

    public default void setExtensionEncoderValue(double val) {}

    public default void setTiltMotorEncoderValue(double val) {}

    public default void setDistanceSimEncoderInput(double distance) {}

    public default void setTiltSimEncoderInput(double angle) {}
}