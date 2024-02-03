// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math; 
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController extensionProfiledPIDControl;
  private final ElevatorFeedforward extensionFeedforward;

  private final PIDController tiltPIDControl;

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

  public Elevator(ElevatorIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        extensionProfiledPIDControl = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);

        tiltPIDControl = new PIDController(0, 0, 0);

        break;
      case SIM:
        extensionProfiledPIDControl = new ProfiledPIDController(40, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.762, 1, 0.0);
        tiltPIDControl = new PIDController(125, 0, 0);

        break;
      case REPLAY:
        extensionProfiledPIDControl = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);

        tiltPIDControl = new PIDController(0, 0, 0);

        break;
      default:
        extensionProfiledPIDControl = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(2.6, 2.6));
        extensionFeedforward = new ElevatorFeedforward(0.0, 0.0, 0, 0.0);

        tiltPIDControl = new PIDController(0, 0, 0);
        break;
    }

    reachState("in");
  }

  @Override
  public void periodic() {
    io.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    double feedback = extensionProfiledPIDControl.calculate(elevatorInputs.elevatorEncoder);
    double feedforward = extensionFeedforward.calculate(extensionProfiledPIDControl.getSetpoint().velocity);
    io.setElevatorVoltage(feedback + feedforward);
    // Units.degreesToRadians(tiltAngleSetPointDeg)

    double pidOutput = tiltPIDControl.calculate(elevatorInputs.absoluteTiltPositionRad.getRadians(), Units.degreesToRadians(tiltAngleSetPointDeg));
    io.setTiltVoltage(pidOutput);

    updateTelemetry();
  }

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

  @AutoLogOutput(key = "Elevator/ConfigPose")
  public Pose3d blankPose() {
    return new Pose3d();
  }

  public void reachExtension(double extensionFeet) {
    extensionProfiledPIDControl.setGoal(Units.feetToMeters(extensionFeet));
  }

  public void setAngle(double angleToSet) {
    tiltAngleSetPointDeg = angleToSet;
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    m_elevatorMech2d.setLength(elevatorInputs.elevatorEncoder);
    Rotation3d angleRot = new Rotation3d(0, -armSim.getAngleRads(), 0);

    Pose3d basePose = new Pose3d(Units.inchesToMeters(-9.6), 0, Units.inchesToMeters(11), angleRot);
    var extensionPose = basePose.transformBy(new Transform3d(new Translation3d(getEleLength(), 0, 0), new Rotation3d()));

    Logger.recordOutput("Elevator/Mechanism3d", 
      basePose, 
      extensionPose, 
      new Pose3d(
        new Translation3d(Constants.LimelightPositions.camPos3.getX(), Constants.LimelightPositions.camPos3.getY(), Constants.LimelightPositions.camPos3.getZ()), 
        new Rotation3d(Constants.LimelightPositions.camPos3.getRotation().getX(), Constants.LimelightPositions.camPos3.getRotation().getY(), Constants.LimelightPositions.camPos3.getRotation().getZ())
      ),
      new Pose3d(
        new Translation3d(Constants.LimelightPositions.camPos2.getX(), Constants.LimelightPositions.camPos2.getY(), Constants.LimelightPositions.camPos2.getZ()), 
        new Rotation3d(Constants.LimelightPositions.camPos2.getRotation().getX(), Constants.LimelightPositions.camPos2.getRotation().getY(), Constants.LimelightPositions.camPos2.getRotation().getZ())
      )
    );
  }

  public void reachTarget(double angle, double extensionFeet) {
    tiltAngleSetPointDeg = angle;
    extensionProfiledPIDControl.setGoal(Units.feetToMeters(extensionFeet));
  }

  public void reachState(String state) {
    switch (state) {
      case "in":
        reachTarget(10, 0);
 
        break;
      case "intake":
        reachTarget(-4, 0.6);

        break;
      case "amp":
        reachTarget(90, 1.425);
        
        break;
      default:
        System.out.println("INVALID STATE");
        break;
    }
  }

  @AutoLogOutput(key = "Elevator/ElevatorLength")
  public double getEleLength() {
    return eleSim.getPositionMeters();
  }

  @AutoLogOutput(key = "Elevator/ElevatorSetpoint")
  public double getExtensionGoal() {
    return extensionProfiledPIDControl.getGoal().position;
  }

  @AutoLogOutput(key = "Elevator/TiltSetpoint")
  public double getTiltSetPoint() {
    return Units.radiansToDegrees(Units.degreesToRadians(tiltPIDControl.getSetpoint()));
  }

  @AutoLogOutput(key = "Elevator/Mechanism2d")
  public Mechanism2d getMechanism() {
    return m_mech2d;
  }
}