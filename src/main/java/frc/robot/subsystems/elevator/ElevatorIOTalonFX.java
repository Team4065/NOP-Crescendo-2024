package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    TalonFX rightTiltMotor = new TalonFX(Constants.ElevatorConstants.rightTiltMotorCANID, "rio");
    TalonFX leftTiltMotor = new TalonFX(Constants.ElevatorConstants.leftTiltMotorCANID, "rio");
    TalonFX extensionMotor = new TalonFX(Constants.ElevatorConstants.extensionMotorCANID, "rio");

    DutyCycleEncoder absTiltEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.absoluteEncoderDIO);

    private final StatusSignal<Double> rightTiltPosRad;
    private final StatusSignal<Double> rightTiltVelc;
    private final StatusSignal<Double> rightTiltApplVolts;
    private final StatusSignal<Double> rightTiltCurrAmp;

    private final StatusSignal<Double> leftTiltPosRad;
    private final StatusSignal<Double> leftTiltVelc;
    private final StatusSignal<Double> leftTiltApplVolts;
    private final StatusSignal<Double> leftTiltCurrAmp;

    private final StatusSignal<Double> elePosRad;
    private final StatusSignal<Double> eleVelc;
    private final StatusSignal<Double> eleApplVolts;
    private final StatusSignal<Double> eleCurrAmp;

    public ElevatorIOTalonFX() {
        var commonMotorConfig = new TalonFXConfiguration();
        commonMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        commonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightTiltMotor.getConfigurator().apply(commonMotorConfig);
        leftTiltMotor.getConfigurator().apply(commonMotorConfig);
        extensionMotor.getConfigurator().apply(commonMotorConfig);

        var configClockwise = new MotorOutputConfigs();
        configClockwise.Inverted = InvertedValue.Clockwise_Positive;
        configClockwise.NeutralMode = NeutralModeValue.Brake;

        rightTiltMotor.getConfigurator().apply(configClockwise);

        var configCounterclockwise = new MotorOutputConfigs();

        configCounterclockwise.Inverted = InvertedValue.CounterClockwise_Positive;
        configCounterclockwise.NeutralMode = NeutralModeValue.Brake;
        leftTiltMotor.getConfigurator().apply(configCounterclockwise);
        extensionMotor.getConfigurator().apply(configCounterclockwise);

        var limitSwitchConfig = new HardwareLimitSwitchConfigs();
        limitSwitchConfig.ForwardLimitEnable = false;
        limitSwitchConfig.ReverseLimitEnable = false;

        extensionMotor.getConfigurator().apply(limitSwitchConfig);


        // absTiltEncoder.setDistancePerRotation(165);

        rightTiltPosRad = rightTiltMotor.getPosition();
        rightTiltVelc = rightTiltMotor.getVelocity();
        rightTiltApplVolts = rightTiltMotor.getMotorVoltage();
        rightTiltCurrAmp = rightTiltMotor.getStatorCurrent();

        leftTiltPosRad = leftTiltMotor.getPosition();
        leftTiltVelc = leftTiltMotor.getVelocity();
        leftTiltApplVolts = leftTiltMotor.getMotorVoltage();
        leftTiltCurrAmp = leftTiltMotor.getStatorCurrent();

        elePosRad = extensionMotor.getPosition();
        eleVelc =  extensionMotor.getVelocity();
        eleApplVolts = extensionMotor.getMotorVoltage();
        eleCurrAmp =  extensionMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rightTiltPosRad,
            rightTiltVelc,
            rightTiltApplVolts,
            rightTiltCurrAmp,
            leftTiltPosRad,
            leftTiltVelc,
            leftTiltApplVolts,
            leftTiltCurrAmp,
            elePosRad,
            eleVelc,
            eleApplVolts,
            eleCurrAmp
        );

        rightTiltMotor.optimizeBusUtilization();
        leftTiltMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rightTiltPosRad,
            rightTiltVelc,
            rightTiltApplVolts,
            rightTiltCurrAmp,
            leftTiltPosRad,
            leftTiltVelc,
            leftTiltApplVolts,
            leftTiltCurrAmp,
            elePosRad,
            eleVelc,
            eleApplVolts,
            eleCurrAmp  
        );

        inputs.absoluteTiltPositionRad = new Rotation2d(Units.degreesToRadians(absTiltEncoder.getDistance()));
        inputs.absoluteDeg = absTiltEncoder.getDistance() + 1;

        inputs.rightTiltPositionRad = new Rotation2d(Units.degreesToRadians(rightTiltPosRad.getValueAsDouble()));
        inputs.rightTiltVelocityRadPerSec = rightTiltVelc.getValueAsDouble();
        inputs.rightTiltAppliedVolts = rightTiltApplVolts.getValueAsDouble();
        inputs.rightTiltCurrentAmps = rightTiltCurrAmp.getValueAsDouble();

        inputs.leftTiltPositionRad = new Rotation2d(Units.degreesToRadians(leftTiltPosRad.getValueAsDouble()));
        inputs.leftTiltVelocityRadPerSec = leftTiltVelc.getValueAsDouble();
        inputs.leftTiltAppliedVolts = leftTiltApplVolts.getValueAsDouble();
        inputs.leftTiltCurrentAmps = leftTiltCurrAmp.getValueAsDouble();

        // Negating the value since going up equals negative value when instead it should be positive
        inputs.elevatorEncoder = Units.inchesToMeters(elePosRad.getValueAsDouble() * 14.75 / 33.69);
        inputs.elevatorPositionRad = new Rotation2d(Units.degreesToRadians(-elePosRad.getValueAsDouble() / 10));
        inputs.elevatorVelocityRadPerSec = -eleVelc.getValueAsDouble() / 10; 
        inputs.elevatorLinearVelocity = 4.5 * (-eleVelc.getValueAsDouble() / 10);

        inputs.elevatorLimitReached = extensionMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
        inputs.elevatorAppliedVolts = eleApplVolts.getValueAsDouble();
        inputs.elevatorCurrentAmps = eleCurrAmp.getValueAsDouble();
    }   

    @Override
    public void setTiltVoltage(double volts) {
        rightTiltMotor.setControl(new VoltageOut(volts));
        leftTiltMotor.setControl(new VoltageOut(volts));
    }

    @Override 
    public void setElevatorVoltage(double volts) {
        extensionMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setExtensionEncoderValue(double value) {
        extensionMotor.setPosition(value);
    }
}
