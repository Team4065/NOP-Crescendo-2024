// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import java.lang.Math; 
import frc.robot.Constants;
import frc.robot.subsystems.vision.IndividualCam;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController extensionProfiledPIDControl;
  private final ElevatorFeedforward extensionFeedforward;

  private final ProfiledPIDController tiltPIDControl;
  private final ArmFeedforward tiltFeedfoward;

  // Simulation only
  private ElevatorSim eleSim = new ElevatorSim(
    Constants.ElevatorConstants.extensionMotorSimGearbox,
    Constants.ElevatorConstants.extensionGearRatio, 
    Constants.ElevatorConstants.extensionCarriageMassKg, 
    Constants.ElevatorConstants.elevatorDrumRadius, 
    Constants.ElevatorConstants.minHeightMeters, 
    Constants.ElevatorConstants.maxHeightMeters, 
    true,
    Constants.ElevatorConstants.minHeightMeters, 
    VecBuilder.fill(0.0)
  );

  private SingleJointedArmSim armSim = new SingleJointedArmSim(
    Constants.ElevatorConstants.tiltMotorSimGearbox,
    Constants.ElevatorConstants.tiltGearRatio,
    SingleJointedArmSim.estimateMOI(eleSim.getPositionMeters() + Constants.ElevatorConstants.fullyInExtension, Constants.ElevatorConstants.armMass),
    eleSim.getPositionMeters(),
    Constants.ElevatorConstants.minAngleRadians,
    Constants.ElevatorConstants.maxAngleRadians,
    false,
    Units.degreesToRadians(10),
    VecBuilder.fill(0.0)
  );
  
  private final Mechanism2d m_mech2d = new Mechanism2d(3, 3);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1.2545, 0.3); 
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d("Elevator", eleSim.getPositionMeters(), Units.radiansToDegrees(armSim.getAngleRads())));

  private double tiltAngleSetPointDeg;
  private double extensionGoal;

  // 13, 7.91

  /* 
    Variables used for "thresholding" 
      For example the arm is at 90 degrees and is fully extended, 
      we want the arm to go down a certain point then start tilting back.
    Maintains the COG
  */
  private boolean extensionThresholdEnabled = false;
  private double extensionThreshold;

  String activationLevel;
  
  String currentState = "in";

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        extensionProfiledPIDControl = new ProfiledPIDController(41, 0, 0.15, new TrapezoidProfile.Constraints(2, 2));
        extensionFeedforward = new ElevatorFeedforward(0.10077, 0.26093, 9.7355, 0.10116);

        tiltPIDControl = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(50, 100), 0.01);
        tiltFeedfoward = new ArmFeedforward(0.17277, 0.3864, 0.041776, 0.0023621);

        break;
      case SIM:
        extensionProfiledPIDControl = new ProfiledPIDController(40, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.762, 1, 0.0);

        tiltPIDControl = new ProfiledPIDController(125, 0, 0, null);
        tiltFeedfoward = new ArmFeedforward(0, 0, 0, 0);

        break;
      case REPLAY:
        extensionProfiledPIDControl = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);

        tiltPIDControl = new ProfiledPIDController(0, 0, 0, null);
        tiltFeedfoward = new ArmFeedforward(0, 0, 0, 0);

        break;
      default:
        extensionProfiledPIDControl = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);

        tiltPIDControl = new ProfiledPIDController(0, 0, 0, null);
        tiltFeedfoward = new ArmFeedforward(0, 0, 0, 0);

        break;
    }

    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    extensionProfiledPIDControl.setGoal(Units.inchesToMeters(extensionGoal));

    double extensionFeedback = extensionProfiledPIDControl.calculate(elevatorInputs.elevatorEncoder);
    double extensionFeedforwardVal = extensionFeedforward.calculate(extensionProfiledPIDControl.getSetpoint().velocity);

    if (elevatorInputs.elevatorLimitReached == true) {
      io.setExtensionEncoderValue(0);
    }

    io.setElevatorVoltage(extensionFeedback + extensionFeedforwardVal);




    double tiltFeedback = tiltPIDControl.calculate(elevatorInputs.absoluteDeg);
    double tiltFeedfowardVal = tiltFeedfoward.calculate(Units.degreesToRadians(tiltPIDControl.getSetpoint().position), tiltPIDControl.getSetpoint().velocity);

    if (elevatorInputs.tiltReached) {
      io.setTiltMotorEncoderValue(-6);
    }
    
    io.setTiltVoltage(tiltFeedback + tiltFeedfowardVal);
    
        

    updateTelemetry();
  }

  public boolean getButtonState() {
    return elevatorInputs.neturalModeButton;
  }

  public boolean isBrake() {
    return elevatorInputs.isBrakeMode;
  }

  public void setBrakeMode(boolean state) {
    io.setBrakeMode(state);
  }

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // public SysIdRoutine extensionRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(null, Volts.of(2), null),
  //   new Mechanism(
  //     (Measure<Voltage> volts) -> {
  //       this.setExtensionVoltage(volts.in(Volts));
  //     }, 
  //     log -> {
  //       log.motor("extension")
  //       .voltage(m_appliedVoltage.mut_replace(this.getExtensionAppliedVoltage(), Volts))
  //       .linearPosition(m_distance.mut_replace(-Units.inchesToMeters(this.getEleLength()), Meters))
  //       .linearVelocity(m_velocity.mut_replace(-Units.inchesToMeters(this.getElevatorLinearVelc()), MetersPerSecond));
  //     }, 
  //     this
  //   )
  // );

  // private final MutableMeasure<Voltage> m_appliedTiltVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutableMeasure<Velocity<Angle>> m_velocityAngle = mutable(DegreesPerSecond.of(0));

  // public SysIdRoutine angleRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(null, Volts.of(3.5), null),
  //   new Mechanism(
  //     (Measure<Voltage> volts) -> {
  //       this.setTiltVoltage(volts.in(Volts));
  //     }, 
  //     log -> {
  //       log.motor("tilt")
  //       .voltage(m_appliedTiltVoltage.mut_replace(this.getTiltAppliedVoltage(), Volts))
  //       .angularPosition(m_angle.mut_replace(elevatorInputs.absoluteDeg, Degrees))
  //       .angularVelocity(m_velocityAngle.mut_replace(elevatorInputs.leftTiltVelocityRadPerSec, DegreesPerSecond));
  //     }, 
  //     this
  //   )
  // );

  @Override
  public void simulationPeriodic() {
    eleSim.setInputVoltage(elevatorInputs.elevatorAppliedVolts);
    eleSim.update(0.02);
    io.setDistanceSimEncoderInput(eleSim.getPositionMeters());


    armSim = new SingleJointedArmSim(
      Constants.ElevatorConstants.tiltMotorSimGearbox,
      Constants.ElevatorConstants.tiltGearRatio,
      SingleJointedArmSim.estimateMOI(eleSim.getPositionMeters() + Constants.ElevatorConstants.fullyInExtension, Constants.ElevatorConstants.armMass),
      eleSim.getPositionMeters(),
      Constants.ElevatorConstants.minAngleRadians,
      Constants.ElevatorConstants.maxAngleRadians,
      false,
      elevatorInputs.absoluteTiltPositionRad.getRadians(),
      VecBuilder.fill(0)
    );

    armSim.setInputVoltage(elevatorInputs.rightTiltAppliedVolts);
    armSim.update(0.02);
    io.setTiltSimEncoderInput(armSim.getAngleRads());
  }

  public void setAngleVoltage(double volts) {
    io.setTiltVoltage(volts);
  }

  public boolean activate(String thresholdType, double thresholdValue, double sensorValue) throws Exception {
    switch (thresholdType) {
      case "above":
        return sensorValue > thresholdValue;        
      case "below":
        return sensorValue < thresholdValue;
      default:
        throw new Exception("INVALID LEVEL!");
    }
  }

  public double getElevatorEncoder() {
    return elevatorInputs.elevatorEncoder;
  }

  public void reachExtension(double extensionInches) {
    extensionProfiledPIDControl.setGoal(extensionInches);
  }

  public void setAngle(double angleToSet) {
    tiltAngleSetPointDeg = angleToSet;
    tiltPIDControl.setGoal(angleToSet);
  }

  @AutoLogOutput(key = "Elevator/TiltGoal")
  public double getAngleGoal() {
    return tiltPIDControl.getGoal().position;
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    m_elevatorMech2d.setLength(elevatorInputs.elevatorEncoder);
  }

  public double getElevatorLinearVelc() {
    return elevatorInputs.elevatorLinearVelocity;
  }
  
  public Pose3d[] getPoses3d() {
    Rotation3d angleRot = new Rotation3d(0, getTiltAngle(), 0);
    Pose3d basePose = new Pose3d(Units.inchesToMeters(-9.6), 0, Units.inchesToMeters(11), angleRot);
    Pose3d extensionPose = basePose.transformBy(new Transform3d(new Translation3d(getEleLength(), 0, 0), new Rotation3d()));

    Pose3d[] posesToReturn = new Pose3d[2];
    posesToReturn[0] = basePose;
    posesToReturn[1] = extensionPose;

    return posesToReturn;
  }
  
  public void reachTarget(double angle, double extensionFeet) {
    tiltAngleSetPointDeg = angle;
    extensionProfiledPIDControl.setGoal(Units.feetToMeters(extensionFeet));
  }

  public void setExtensionVoltage(double volts) {
    io.setElevatorVoltage(volts);
  }

  public double getExtensionAppliedVoltage() {
    return elevatorInputs.elevatorAppliedVolts;
  }

  public double getTiltAppliedVoltage() {
    return elevatorInputs.rightTiltAppliedVolts;
  }

  public void reachState(String state) {
    switch (state) {
      case "in":
        extensionThresholdEnabled = false;
        reachTarget(10, 0);
        currentState = "in";

        break;
      case "intake":
        extensionThresholdEnabled = false;

        if (currentState == "in") {
          activationLevel = "above";
          extensionThreshold = 0.19;
        } else {
          activationLevel = "below";
          extensionThreshold = 0.2;
        }
        
        extensionThresholdEnabled = true;
        reachTarget(-4, 0.6);  
        currentState = "intake";

        break;
      case "amp":
        extensionThresholdEnabled = false;
        reachTarget(90, 1);
        currentState = "amp";
        
        break;
      default:
        System.out.println("INVALID STATE");
        break;
    }
  }

  @AutoLogOutput(key = "Elevator/ElevatorLength")
  public double getEleLength() {
    switch (Constants.currentMode) {
      case REAL:
        return elevatorInputs.elevatorEncoder;

      case SIM:
        return eleSim.getPositionMeters();

      default:
        return 0;
    }
  }

  @AutoLogOutput(key = "Elevator/TiltAngle")
  public double getTiltAngle() {
    switch (Constants.currentMode) {
      case REAL:
        return -elevatorInputs.absoluteDeg;
      
      case SIM:
        return -armSim.getAngleRads();

      default:
        return 0;
    }
  }

  public double getAngleDeg() {
    return elevatorInputs.absoluteDeg;
  }

  @AutoLogOutput(key = "Elevator/ElevatorSetpoint")
  public double getExtensionGoal() {
    return extensionProfiledPIDControl.getSetpoint().position;
  }

  @AutoLogOutput(key = "Elevator/TiltSetpoint")
  public double getTiltSetPoint() {
    return tiltPIDControl.getSetpoint().position;
  }

  @AutoLogOutput(key = "Elevator/ExtensionError")
  public double getErrorExtension() {
    return Math.abs(extensionProfiledPIDControl.getPositionError());
  }

  @AutoLogOutput(key = "Elevator/TiltError")
  public double getErrorTilt() {
    return Math.abs(tiltPIDControl.getPositionError());
  }

  @AutoLogOutput(key = "Elevator/Mechanism2d")
  public Mechanism2d getMechanism() {
    return m_mech2d;
  }
}