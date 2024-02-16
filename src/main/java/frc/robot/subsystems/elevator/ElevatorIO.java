package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

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

        public Rotation2d absoluteTiltPositionRad = new Rotation2d();
        public double absoluteDeg = 0.0;

        public double elevatorEncoder = 0;
        public Rotation2d elevatorPositionRad = new Rotation2d();
        public double elevatorVelocityRadPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;
        public boolean elevatorLimitReached = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setTiltVoltage(double volts) {}

    public default void setElevatorVoltage (double volts) {}

    public default void setTiltBrakeMode(boolean enable) {}

    public default void setElevatorBrakeMode(boolean enable) {}

    public default void setExtensionEncoderValue(double val) {}

    public default void setDistanceSimEncoderInput(double distance) {}

    public default void setTiltSimEncoderInput(double angle) {}
}