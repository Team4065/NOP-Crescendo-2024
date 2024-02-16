package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
    TalonFX rightTelescopeMotor = new TalonFX(Constants.ClimberConstants.rightTelescopeCANID, "rio");
    TalonFX leftTelescopeMotor = new TalonFX(Constants.ClimberConstants.leftTelescopeCANID, "rio");

    Compressor compressorPCM = new Compressor(Constants.ClimberConstants.pcmCANID, PneumaticsModuleType.REVPH);
    Solenoid rightRatchet = new Solenoid(PneumaticsModuleType.REVPH, Constants.ClimberConstants.rightRatchetPort);
    Solenoid leftRatchet = new Solenoid(PneumaticsModuleType.REVPH, Constants.ClimberConstants.leftRatchetPort);

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

        rightTelescopeMotor.getConfigurator().apply(configClockwise);

        var configCounterClockwise = new MotorOutputConfigs();
        configClockwise.Inverted = InvertedValue.CounterClockwise_Positive;

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

        compressorPCM.enableDigital();

        rightRatchet.set(false);
        leftRatchet.set(false);
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
        inputs.rightTelescopeRatchet = rightRatchet.get();
        inputs.leftTelescopeRatchet = leftRatchet.get();
    }

    @Override
    public void setSpeed(double speed) {
        rightRatchet.set((speed != 0 ? true : false));
        leftRatchet.set((speed != 0 ? true : false));

        rightTelescopeMotor.set(speed);
        leftTelescopeMotor.set(speed);
    }

    @Override
    public void setRatchet(boolean state) {
        rightRatchet.set(state);
        leftRatchet.set(state);
    }
}
