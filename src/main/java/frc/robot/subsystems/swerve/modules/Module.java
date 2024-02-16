package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private int index;

  private final SimpleMotorFeedforward driveFeedForward; // FeedForward is used to calculate velocity
  private final PIDController drivePID;
  private PIDController turnPID;

  private Rotation2d angleSetPoint = null;
  private Double speedSetPoint = null; // Open-loop, not PID
  private Rotation2d relativeTurnOffset = new Rotation2d(0); // relative + offset = absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation a.k.a change

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    switch (Constants.currentMode) {
      case REAL:
        // NEED TO TUNE PID
        driveFeedForward = new SimpleMotorFeedforward(0.03412, 0.14132);
        drivePID = new PIDController(0, 0, 0);
        turnPID = new PIDController(3.5, 0, 0); 
        break;
      case REPLAY:
        driveFeedForward = new SimpleMotorFeedforward(0.1, 0.13);
        drivePID = new PIDController(0.05, 0.0, 0.0);
        turnPID = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        driveFeedForward = new SimpleMotorFeedforward(0.0, 0.13);
        drivePID = new PIDController(0.1, 0.0, 0.0);
        turnPID = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        driveFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
        drivePID = new PIDController(0.0, 0.0, 0.0);
        turnPID = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    turnPID.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

    if (relativeTurnOffset != null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
      relativeTurnOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition); // relative = absolute - relative position
    }

    if (angleSetPoint != null) {
      io.setTurnVoltage(turnPID.calculate(getAngle().getRadians(), angleSetPoint.getRadians()));
      // Closed loop control of speed of the module
      if (speedSetPoint != null) {
        // Scale velocity based on turn error

        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetPoint * Math.cos(turnPID.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / Constants.SwerveConstants.wheel_radius_meters; // calculate the velocity setpoint
        io.setDriveVoltage(driveFeedForward.calculate(velocityRadPerSec) + drivePID.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }
  }

  // Optimize the state, makes sure that the swerve when trying to achieve the angle setpoint it
  // doesnt take the long way
  public SwerveModuleState runSetPoint(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    angleSetPoint = optimizedState.angle;
    speedSetPoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  // Return angle of module
  public Rotation2d getAngle() {
    if (relativeTurnOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(relativeTurnOffset);
    }
  }

  // Makes the heading to 0 so we can only use drive for characterization
  public void runCharacterization(double volts) {
    angleSetPoint = new Rotation2d();

    io.setDriveVoltage(volts);
    speedSetPoint = null;
  }

  public void stop() {
    io.setTurnVoltage(0);
    io.setDriveVoltage(0);

    angleSetPoint = null;
    speedSetPoint = null;
  }

  public void setBrakeMode(boolean enable) {
    io.setTurnBrakeMode(enable);
    io.setDriveBrakeMode(enable);
  }

  // Get position of drive in module
  public double getPosMeters() {
    return inputs.drivePositionRad * Constants.SwerveConstants.wheel_radius_meters;
  }

  // Get velocity of module
  public double getVelcMeters() {
    return inputs.driveVelocityRadPerSec * Constants.SwerveConstants.wheel_radius_meters;
  }

  public SwerveModulePosition getModulePos() {
    return new SwerveModulePosition(getPosMeters(), getAngle());
  }

  public SwerveModulePosition getPosDelta() {
    var delta = new SwerveModulePosition(getPosMeters() - lastPositionMeters, getAngle());
    lastPositionMeters = getPosMeters();

    return delta;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelcMeters(), getAngle());
  }

  public double getCharacterizationVelc() {
    return inputs.driveVelocityRadPerSec;
  }
}
