package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    DCMotorSim rightTiltMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.025);
    DCMotorSim leftTiltMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.025);
    DCMotorSim elevatorMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.025);

    // Encoder extensionEncoder = new Encoder(Constants.ElevatorConstants.encoderAChannel, Constants.ElevatorConstants.encoderBChannel);
    // EncoderSim extensionEncoderSim = new EncoderSim(extensionEncoder);

    // Encoder tiltEncoder = new Encoder(Constants.ElevatorConstants.encoderCChannel, Constants.ElevatorConstants.encoderDChannel);
    // EncoderSim tiltEncoderSim = new EncoderSim(tiltEncoder);

    double appliedRightTiltMotorVolts = 0.0;
    double appliedLeftTiltMotorVolts = 0.0;
    double appliedElevatorMotorVolts = 0.0;


    public ElevatorIOSim() {
        // extensionEncoderSim.setDistancePerPulse(Constants.ElevatorConstants.elevatorEncoderDistPerPulse);
        // tiltEncoderSim.setDistancePerPulse(Constants.ElevatorConstants.tiltEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        rightTiltMotor.update(0.02);
        leftTiltMotor.update(0.02);
        elevatorMotor.update(0.02);

        inputs.rightTiltPositionRad = rightTiltMotor.getAngularPositionRad();
        inputs.rightTiltVelocityRadPerSec = rightTiltMotor.getAngularVelocityRadPerSec();
        inputs.rightTiltAppliedVolts = appliedRightTiltMotorVolts;
        inputs.rightTiltCurrentAmps = Math.abs(rightTiltMotor.getCurrentDrawAmps());

        inputs.leftTiltPositionRad = new Rotation2d(leftTiltMotor.getAngularPositionRad());
        inputs.leftTiltVelocityRadPerSec = leftTiltMotor.getAngularVelocityRadPerSec();
        inputs.leftTiltAppliedVolts = appliedLeftTiltMotorVolts;
        inputs.leftTiltCurrentAmps = Math.abs(leftTiltMotor.getCurrentDrawAmps());

        // inputs.absoluteTiltPositionRad = new Rotation2d(tiltEncoderSim.getDistance());

        // inputs.elevatorEncoder = extensionEncoderSim.getDistance();
        inputs.elevatorPositionRad = new Rotation2d(elevatorMotor.getAngularPositionRad());
        inputs.elevatorVelocityRadPerSec = elevatorMotor.getAngularVelocityRadPerSec();
        inputs.elevatorAppliedVolts = appliedElevatorMotorVolts;
        inputs.elevatorCurrentAmps = Math.abs(elevatorMotor.getCurrentDrawAmps());
    }

    @Override
    public void setTiltVoltage(double volts) {
        appliedRightTiltMotorVolts = MathUtil.clamp(volts, -12.0, 12);
        appliedLeftTiltMotorVolts = MathUtil.clamp(-volts, -12.0, 12);

        rightTiltMotor.setInputVoltage(appliedRightTiltMotorVolts);
        leftTiltMotor.setInputVoltage(appliedLeftTiltMotorVolts);
    }

    @Override
    public void setElevatorVoltage(double volts) {
        appliedElevatorMotorVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotor.setInputVoltage(appliedElevatorMotorVolts);
    }
    
    @Override 
    public void setDistanceSimEncoderInput(double distance) {
        // extensionEncoderSim.setDistance(distance);
    }

    @Override
    public void setTiltSimEncoderInput(double angleRad) {
        // tiltEncoderSim.setDistance(angleRad);  
    }
}