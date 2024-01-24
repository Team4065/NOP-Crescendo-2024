// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  private ProfiledPIDController profiledPIDControl = new ProfiledPIDController(5, 1, 0, new TrapezoidProfile.Constraints(2.45, 2.45));
  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 0.762, 0.762, 0.0);


  // Simulation only
  private DCMotor elevatorMotor = DCMotor.getFalcon500(1);
  private ElevatorSim eleSim = new ElevatorSim(
    elevatorMotor,
    Constants.ElevatorConstants.gearRatio, 
    Constants.ElevatorConstants.carriageMassKg, 
    Constants.ElevatorConstants.elevatorDrumRadius, 
    Constants.ElevatorConstants.minHeightMeters - Constants.ElevatorConstants.fullyInExtension, 
    Constants.ElevatorConstants.maxHeightMeters - Constants.ElevatorConstants.fullyInExtension, 
    true, 
    Constants.ElevatorConstants.minHeightMeters - Constants.ElevatorConstants.fullyInExtension, 
    VecBuilder.fill(0.0)
  );
  
  private final Mechanism2d m_mech2d = new Mechanism2d(3, 3);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1.2545, 0.3);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d("Elevator", eleSim.getPositionMeters(), 10));

  public Elevator(ElevatorIO io) {
    this.io = io;
    if (Constants.currentMode == Constants.Mode.SIM) {
      io.setConversionRateSimEncoder(Constants.ElevatorConstants.elevatorEncoderDistPerPulse);
      System.out.println("IN SIM");
    } 
  }

  @Override
  public void periodic() {
    io.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    double feedback = profiledPIDControl.calculate(elevatorInputs.elevatorEncoder);
    double feedforward = m_feedforward.calculate(profiledPIDControl.getSetpoint().velocity);
    io.setElevatorVoltage(feedback + feedforward);
  }

  @Override
  public void simulationPeriodic() {
    eleSim.setInputVoltage(elevatorInputs.elevatorAppliedVolts);
    eleSim.update(0.02);
    io.setDistanceSimEncoderInput(eleSim.getPositionMeters());
    updateTelemetry();
    
  }

  public void reachExtension(double extensionFeet) {
    profiledPIDControl.setGoal(Units.feetToMeters(extensionFeet) - Units.inchesToMeters(9.21) - Constants.ElevatorConstants.fullyInExtension);
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(elevatorInputs.elevatorEncoder);
  }

  @AutoLogOutput(key = "Elevator/ElevatorLength")
  public double getEleLength() {
    return eleSim.getPositionMeters();
  }

  @AutoLogOutput(key = "Elevator/ElevatorSetpoint")
  public double getExtensionGoal() {
    return profiledPIDControl.getGoal().position;
  }

  @AutoLogOutput(key = "Elevator/EleMechanismVis")
  public Mechanism2d getMechanism() {
    return m_mech2d;
  }

  @AutoLogOutput(key = "Elevator/Mechanism3d")
  public Pose3d getElevatorRelativePose() {
    Pose3d startingPose = new Pose3d(0, 0, Units.inchesToMeters(-2.5), new Rotation3d(0, Units.degreesToRadians(-10), 0));
    return startingPose.transformBy(new Transform3d(getEleLength(), 0, 0, new Rotation3d()));
  }
}
