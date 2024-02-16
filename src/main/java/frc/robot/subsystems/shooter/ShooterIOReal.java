package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    TalonFX topRollerMotor = new TalonFX(Constants.ShooterConstants.topRollerCANID, "rio");
    TalonFX bottomRollerMotor = new TalonFX(Constants.ShooterConstants.bottomRollerCANID, "rio");

    CANSparkMax intakeMotor = new CANSparkMax(Constants.ShooterConstants.intakeMotorCANID, MotorType.kBrushless);
    DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreakDIO);


    private final StatusSignal<Double> topRollerPosRad;
    private final StatusSignal<Double> topRollerVelc;
    private final StatusSignal<Double> topRollerApplVolts;
    private final StatusSignal<Double> topRollerCurrAmp;

    private final StatusSignal<Double> bottomRollerPosRad;
    private final StatusSignal<Double> bottomRollerVelc;
    private final StatusSignal<Double> bottomRollerApplVolts;
    private final StatusSignal<Double> bottomRollerCurrAmp;

    public ShooterIOReal() {
        topRollerPosRad = topRollerMotor.getPosition();
        topRollerVelc = topRollerMotor.getVelocity();
        topRollerApplVolts = topRollerMotor.getMotorVoltage();
        topRollerCurrAmp = topRollerMotor.getStatorCurrent();

        bottomRollerPosRad = bottomRollerMotor.getPosition();
        bottomRollerVelc = bottomRollerMotor.getVelocity();
        bottomRollerApplVolts = bottomRollerMotor.getMotorVoltage();
        bottomRollerCurrAmp = bottomRollerMotor.getStatorCurrent();

        intakeMotor.setInverted(true);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            topRollerPosRad,
            topRollerVelc,
            topRollerApplVolts,
            topRollerCurrAmp,
            bottomRollerPosRad,
            bottomRollerVelc,
            bottomRollerApplVolts,
            bottomRollerCurrAmp
        );

        topRollerMotor.optimizeBusUtilization();
        bottomRollerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            topRollerPosRad,
            topRollerVelc,
            topRollerApplVolts,
            topRollerCurrAmp,
            bottomRollerPosRad,
            bottomRollerVelc,
            bottomRollerApplVolts,
            bottomRollerCurrAmp
        );

        inputs.topRollerPositionRad = topRollerPosRad.getValueAsDouble();
        inputs.topRollerVelocityRadPerSec = topRollerVelc.getValueAsDouble();
        inputs.topRollerAppliedVolts = topRollerApplVolts.getValueAsDouble();
        inputs.topRollerCurrentAmps = topRollerCurrAmp.getValueAsDouble();

        inputs.bottomRollerPositionRad = bottomRollerPosRad.getValueAsDouble();
        inputs.bottomRollerVelocityRadPerSec = bottomRollerVelc.getValueAsDouble();
        inputs.bottomRollerAppliedVolts = bottomRollerApplVolts.getValueAsDouble();
        inputs.bottomRollerCurrentAmps = bottomRollerCurrAmp.getValueAsDouble();

        inputs.intakePositionRad = intakeMotor.getEncoder().getPosition();
        inputs.intakeVelocityRadPerSec = intakeMotor.getEncoder().getVelocity();
        inputs.intakeAppliedVolts = intakeMotor.getVoltageCompensationNominalVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.beamBreak = !(beamBreak.get());
    }

    @Override
    public void setShooterVoltage(double volts) {
        topRollerMotor.setControl(new VoltageOut(volts));
        bottomRollerMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
