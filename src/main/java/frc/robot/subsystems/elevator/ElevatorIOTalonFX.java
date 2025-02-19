package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    TalonFX rightTiltMotor;
    TalonFX leftTiltMotor;
    TalonFX extensionMotor;

    DigitalInput buttonState = new DigitalInput(Constants.ElevatorConstants.neutralModeButton);

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


    final MotionMagicVoltage tiltRequest = new MotionMagicVoltage(0);
    final MotionMagicDutyCycle extensionRequest = new MotionMagicDutyCycle(0);

    boolean isBrake;

    double rotSetpointField;

    public ElevatorIOTalonFX() {
        rightTiltMotor = new TalonFX(Constants.ElevatorConstants.rightTiltMotorCANID, "rio");
        leftTiltMotor = new TalonFX(Constants.ElevatorConstants.leftTiltMotorCANID, "rio");
        extensionMotor = new TalonFX(Constants.ElevatorConstants.extensionMotorCANID, "rio");

        leftTiltMotor.setControl(new Follower(rightTiltMotor.getDeviceID(), true));

        var commonMotorConfig = new TalonFXConfiguration();
        commonMotorConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        commonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightTiltMotor.getConfigurator().apply(commonMotorConfig);
        leftTiltMotor.getConfigurator().apply(commonMotorConfig);
        extensionMotor.getConfigurator().apply(commonMotorConfig);

        var configClockwise = new MotorOutputConfigs();
        configClockwise.Inverted = InvertedValue.Clockwise_Positive;
        configClockwise.NeutralMode = NeutralModeValue.Brake;


        var configCounterclockwise = new MotorOutputConfigs();

        configCounterclockwise.Inverted = InvertedValue.CounterClockwise_Positive;
        configCounterclockwise.NeutralMode = NeutralModeValue.Brake;

        rightTiltMotor.getConfigurator().apply(configCounterclockwise);
        leftTiltMotor.getConfigurator().apply(configClockwise);
        extensionMotor.getConfigurator().apply(configCounterclockwise);

        var limitSwitchConfig = new HardwareLimitSwitchConfigs();
        limitSwitchConfig.ForwardLimitEnable = false;
        limitSwitchConfig.ReverseLimitEnable = false;

        rightTiltMotor.getConfigurator().apply(limitSwitchConfig);
        extensionMotor.getConfigurator().apply(limitSwitchConfig);

        var feedbackConfigs = new TalonFXConfiguration();

        var slot0config = feedbackConfigs.Slot0;
        slot0config.kS = 0.13857;   
        slot0config.kG = 0.040253;
        slot0config.kV = 0.10339;
        slot0config.kA = 0.0058069;
        slot0config.kP = 1.75;
        slot0config.kI = 0;
        slot0config.kD = 0;

        var motionMagicConfigs = feedbackConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicAcceleration = 120;
        motionMagicConfigs.MotionMagicJerk = 1600;

        rightTiltMotor.getConfigurator().apply(feedbackConfigs);
        

        isBrake = true;

        rightTiltPosRad = rightTiltMotor.getRotorPosition();
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

        rotSetpointField = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
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

        BaseStatusSignal.setUpdateFrequencyForAll(100, rightTiltPosRad);

        rightTiltMotor.optimizeBusUtilization();
        leftTiltMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
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

        BaseStatusSignal.refreshAll(rightTiltPosRad);

        inputs.absoluteDeg = (rightTiltPosRad.getValueAsDouble() + 2.2222) * 2.42 + -5;
        inputs.absoluteVelc = rightTiltPosRad.getValueAsDouble() * 2.42;

        inputs.rightTiltPositionRad = rightTiltPosRad.getValueAsDouble();
        inputs.rightTiltVelocityRadPerSec = rightTiltVelc.getValueAsDouble();
        inputs.rightTiltAppliedVolts = rightTiltApplVolts.getValueAsDouble();
        inputs.rightTiltCurrentAmps = rightTiltCurrAmp.getValueAsDouble();

        inputs.leftTiltPositionRad = new Rotation2d(Units.degreesToRadians(leftTiltPosRad.getValueAsDouble()));
        inputs.leftTiltVelocityRadPerSec = leftTiltVelc.getValueAsDouble();
        inputs.leftTiltAppliedVolts = leftTiltApplVolts.getValueAsDouble();
        inputs.leftTiltCurrentAmps = leftTiltCurrAmp.getValueAsDouble();

        // Negating the value since going up equals negative value when instead it should be positive
        inputs.elevatorEncoder = Units.inchesToMeters(elePosRad.getValueAsDouble() * 11.2 / 42.155);
        inputs.elevatorPositionRad = -elePosRad.getValueAsDouble();
        inputs.elevatorVelocityRadPerSec = -eleVelc.getValueAsDouble() / 10; 
        inputs.elevatorLinearVelocity = 4.5 * (-eleVelc.getValueAsDouble() / 10);

        inputs.elevatorLimitReached = extensionMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
        inputs.elevatorAppliedVolts = eleApplVolts.getValueAsDouble();
        inputs.elevatorCurrentAmps = eleCurrAmp.getValueAsDouble();

        inputs.tiltGoal = rotSetpointField;

        // TRUE -- NOT PRESSED, FALSE -- PRESSED
        inputs.neturalModeButton = buttonState.get();
        inputs.isBrakeMode = isBrake;

        inputs.tiltReached = rightTiltMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
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
    public void setElevatorCustomControl(ControlRequest request) {
        extensionMotor.setControl(request);
    }

    @Override
    public void setExtensionEncoderValue(double value) {
        extensionMotor.setPosition(value);
    }

    @Override
    public void runTiltSetpoint(double rotSetpoint) {
        rotSetpointField = rotSetpoint;
        rightTiltMotor.setControl(tiltRequest.withPosition(rotSetpoint));
        leftTiltMotor.setControl(new Follower(rightTiltMotor.getDeviceID(), true));
    }

    @Override
    public void runElevatorSetpoint(double rotExtension) {
        // extensionMotor.setControl(extensionRequest.withPosition(rotExtension));
    }

    @Override
    public void setBrakeMode(boolean state) {
        if (state) {
            var configClockwise = new MotorOutputConfigs();
            configClockwise.Inverted = InvertedValue.Clockwise_Positive;
            configClockwise.NeutralMode = NeutralModeValue.Brake;


            var configCounterclockwise = new MotorOutputConfigs();

            configCounterclockwise.Inverted = InvertedValue.CounterClockwise_Positive;
            configCounterclockwise.NeutralMode = NeutralModeValue.Brake;

            rightTiltMotor.getConfigurator().apply(configCounterclockwise);
            leftTiltMotor.getConfigurator().apply(configClockwise);

            extensionMotor.getConfigurator().apply(configCounterclockwise);

            isBrake = true;
        } else if (state == false) {
            var configClockwise = new MotorOutputConfigs();
            configClockwise.Inverted = InvertedValue.Clockwise_Positive;
            configClockwise.NeutralMode = NeutralModeValue.Coast;


            var configCounterclockwise = new MotorOutputConfigs();

            configCounterclockwise.Inverted = InvertedValue.CounterClockwise_Positive;
            configCounterclockwise.NeutralMode = NeutralModeValue.Coast;

            rightTiltMotor.getConfigurator().apply(configCounterclockwise);
            leftTiltMotor.getConfigurator().apply(configClockwise);

            extensionMotor.getConfigurator().apply(configCounterclockwise);

            isBrake = false;
        }
    }

    @Override
    public void setTiltMotorEncoderValue(double degrees) {
        rightTiltMotor.setPosition(Units.degreesToRotations(degrees) * 133.3333333333333333333);
        leftTiltMotor.setPosition(Units.degreesToRotations(degrees) * 133.3333333333333333333);
    }
}
