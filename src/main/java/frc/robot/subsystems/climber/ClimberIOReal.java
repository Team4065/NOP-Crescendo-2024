package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
    TalonFX rightTelescopeMotor = new TalonFX(Constants.ClimberConstants.rightTelescopeCANID, "rio");
    TalonFX leftTelescopeMotor = new TalonFX(Constants.ClimberConstants.leftTelescopeCANID, "rio");

    Compressor compressorPCM = new Compressor(Constants.ClimberConstants.pcmCANID, PneumaticsModuleType.REVPH);
    // DoubleSolenoid ratchet = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    Solenoid right = new Solenoid(Constants.ClimberConstants.pcmCANID, PneumaticsModuleType.REVPH, 0);
    Solenoid left = new Solenoid(Constants.ClimberConstants.pcmCANID, PneumaticsModuleType.REVPH, 1);

    private final StatusSignal<Double> rightTelePosRad;
    private final StatusSignal<Double> rightTeleVelc;
    private final StatusSignal<Double> rightTeleApplVolts;
    private final StatusSignal<Double> rightTeleCurrAmp;

    private final StatusSignal<Double> leftTelePosRad;
    private final StatusSignal<Double> leftTeleVelc;
    private final StatusSignal<Double> leftTeleApplVolts;
    private final StatusSignal<Double> leftTeleCurrAmp;

    public ClimberIOReal() {
        var configClockwise = new MotorOutputConfigs();
        configClockwise.Inverted = InvertedValue.Clockwise_Positive;

        configClockwise.NeutralMode = NeutralModeValue.Brake;
        rightTelescopeMotor.getConfigurator().apply(configClockwise);

        var configCounterClockwise = new MotorOutputConfigs();
        configCounterClockwise.NeutralMode = NeutralModeValue.Brake;
        configCounterClockwise.Inverted = InvertedValue.Clockwise_Positive;

        leftTelescopeMotor.getConfigurator().apply(configCounterClockwise);

        rightTelePosRad = rightTelescopeMotor.getPosition();
        rightTeleVelc = rightTelescopeMotor.getVelocity();
        rightTeleApplVolts = rightTelescopeMotor.getMotorVoltage();
        rightTeleCurrAmp = rightTelescopeMotor.getStatorCurrent();

        leftTelePosRad = leftTelescopeMotor.getPosition();
        leftTeleVelc = leftTelescopeMotor.getVelocity();
        leftTeleApplVolts = leftTelescopeMotor.getMotorVoltage();
        leftTeleCurrAmp = leftTelescopeMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rightTelePosRad,
            rightTeleVelc,
            rightTeleApplVolts,
            rightTeleCurrAmp,
            leftTelePosRad,
            leftTeleVelc,
            leftTeleApplVolts,
            leftTeleCurrAmp
        );

        rightTelescopeMotor.optimizeBusUtilization();
        leftTelescopeMotor.optimizeBusUtilization();

        // compressorPCM.enableHybrid(50, 60);
        compressorPCM.enableDigital();


        // TRUE = MOTOR MOVEMENT, FALSE = NO MOTOR MOVEMENT
        // ratchet.set(Value.kForward);
        setRatchet(true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rightTelePosRad,
            rightTeleVelc,
            rightTeleApplVolts,
            rightTeleCurrAmp,
            leftTelePosRad,
            leftTeleVelc,
            leftTeleApplVolts,
            leftTeleCurrAmp
        );

        inputs.rightTelescopePositionRad = rightTelePosRad.getValueAsDouble();
        inputs.rightTelescopeVelocityRadPerSec = rightTeleVelc.getValueAsDouble();
        inputs.rightTelescopeAppliedVolts = rightTeleApplVolts.getValueAsDouble();
        inputs.rightTelescopeCurrentAmps = rightTeleCurrAmp.getValueAsDouble();

        inputs.leftTelescopePositionRad = leftTelePosRad.getValueAsDouble();
        inputs.leftTelescopeVelocityRadPerSec = leftTeleVelc.getValueAsDouble();
        inputs.leftTelescopeAppliedVolts = leftTeleApplVolts.getValueAsDouble();
        inputs.leftTelescopeCurrentAmps = leftTeleCurrAmp.getValueAsDouble();

        inputs.compressorPSI = compressorPCM.getPressure();
        inputs.rightTelescopeRatchet = false;
        inputs.leftTelescopeRatchet = false;
    }

    @Override
    public void setSpeed(double speed) {
        rightTelescopeMotor.set(speed);
        leftTelescopeMotor.set(speed);
    }

    @Override
    public void setRatchet(boolean state) {
        right.set(state);
        left.set(!state);
    }
}
