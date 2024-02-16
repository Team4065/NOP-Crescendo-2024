package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double rightTelescopePositionRad = 0.0;
        public double rightTelescopeVelocityRadPerSec = 0.0;
        public double rightTelescopeAppliedVolts = 0.0;
        public double rightTelescopeCurrentAmps = 0.0;

        public double leftTelescopePositionRad = 0.0;
        public double leftTelescopeVelocityRadPerSec = 0.0;
        public double leftTelescopeAppliedVolts = 0.0;
        public double leftTelescopeCurrentAmps = 0.0;

        public double compressorPSI = 0.0;

        // false = in, true = out
        public boolean rightTelescopeRatchet = false;
        public boolean leftTelescopeRatchet = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setSpeed(double speed) {}

    public default void setRatchet(boolean state) {}
}
