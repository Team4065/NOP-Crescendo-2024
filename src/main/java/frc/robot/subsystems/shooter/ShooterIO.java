package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public boolean beamBreak = false;

        public double topRollerPositionRad = 0.0;
        public double topRollerVelocityRadPerSec = 0.0;
        public double topRollerAppliedVolts = 0.0;
        public double topRollerCurrentAmps = 0.0;

        public double bottomRollerPositionRad = 0.0;
        public double bottomRollerVelocityRadPerSec = 0.0;
        public double bottomRollerAppliedVolts = 0.0;
        public double bottomRollerCurrentAmps = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}
    
    public default void setShooterVoltage(double volts) {}
}
