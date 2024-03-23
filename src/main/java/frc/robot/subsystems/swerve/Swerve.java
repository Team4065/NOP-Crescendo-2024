// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

  private double max_drive_speed = Units.feetToMeters(Constants.SwerveConstants.MAX_SPEED_FEET);
  static final double track_width_x = Units.inchesToMeters(28);
  static final double track_width_y = Units.inchesToMeters(28);
  static final double drivetrain_radius = Math.hypot(track_width_x / 2, track_width_y / 2);
  private double max_angular_speed = max_drive_speed / drivetrain_radius;

  final GyroIO gyroIO;
  final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  final Module[] modules = new Module[4];

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Rotation2d rawGyroRotation = new Rotation2d();

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  Pose2d pose = new Pose2d(0, 0, new Rotation2d());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final PIDController headingPID;
  public double headingFeedbackVal = 0;

  /** Creates a new Swerve. */
  public Swerve(GyroIO gyroIO, ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR) {
    this.gyroIO = gyroIO; // Initialize gyro
    // Initialize modules
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFR, 1);
    modules[2] = new Module(moduleBL, 2);
    modules[3] = new Module(moduleBR, 3);

    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose,
      () -> kinematics.toChassisSpeeds(getModuleStates()),
      this::runVelc,
      new HolonomicPathFollowerConfig(
        3.1,
        drivetrain_radius,
        new ReplanningConfig()
      ),
      () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
      this
    );
        
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      // Log the active path in AdvantageKit
      Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));

      // Display the active path on shuffleboard
      if (!LoggedDriverStation.getDSData().autonomous) {
        Trajectory trajToDisplay = (activePath.size()) > 0 ? TrajectoryGenerator.generateTrajectory(activePath, new TrajectoryConfig(4 ,3)) : new Trajectory();
        Constants.displayField.getObject("Field").setTrajectory(trajToDisplay);
      }
    });

    switch (Constants.currentMode) {
      case REAL: 
        headingPID = new PIDController(2, 0, 0);
        break;
      
      case SIM:
        headingPID = new PIDController(2.3, 0.01, 0.009);
        break;

      default:
        headingPID = new PIDController(0, 0, 0);
        break;
    }

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Pose2d getFusedOdometry() {
    return RobotContainer.m_vision.getEstimatedPose();
  }

  public void setFusedOdometry(Pose2d pose) {
    RobotContainer.m_vision.setPose(pose);
    setPose(pose);
  }

  public void resetGyro() {
    gyroIO.resetGyro();
  }

  public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  public SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
  public SwerveModulePosition[] modulePos = new SwerveModulePosition[4];

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    // Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      // Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      // Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    modulePositions = getModulePos();
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    // if (gyroInputs.isConnected) {
      // Use the real gyro angle
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        rawGyroRotation = gyroInputs.yawPos;
      } else {
        rawGyroRotation = gyroInputs.yawPos.plus(new Rotation2d(Units.degreesToRadians(180)));
      }
    // } else {
    //   // Use the angle delta from the kinematics and module deltas
    //   Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    //   rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    // }

    poseEstimator.update(rawGyroRotation, modulePositions);

    Constants.displayField.setRobotPose(getPose());
  }

  public SwerveModulePosition[] getModulePos() {
    for (int i = 0; i < 4; i++) {
      modulePos[i] = modules[i].getModulePos();
    }
    
    return modulePos;
  }

  // Your field-relative control
  public void runVelc(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, max_drive_speed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetPoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return max_drive_speed;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return max_angular_speed;
  }


  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Swerve/HeadingSetpoint")
  public double getHeadingSetpoint() {
    return headingPID.getSetpoint();
  }
  
  public double distanceFormula(Pose2d point1, Pose2d point2) {
    double xPos1 = point1.getX();
    double yPos1 = point1.getY();

    double xPos2 = point2.getX();
    double yPos2 = point2.getY();

    return Math.sqrt(Math.pow(xPos2 - xPos1, 2) + Math.pow(yPos2 - yPos1, 2));
  }

  public double getDistanceFromSpeaker(String allianceColor) {
    switch (allianceColor) {
      case "blue":
        return distanceFormula(
          new Pose2d(
            Constants.FieldConstants.blueSpeakerReferencePoint.getX(),
            Constants.FieldConstants.blueSpeakerReferencePoint.getY(),
            new Rotation2d()
          ),
          pose
        );
      case "red":
        return distanceFormula(
          new Pose2d(
            Constants.FieldConstants.redSpeakerReferencePoint.getX(),
            Constants.FieldConstants.redSpeakerReferencePoint.getY(),
            new Rotation2d()
          ),
          pose
        );
      default:
        return 0;
    }
  }

  public void setSpeed(double max_speed_feet) {
    max_drive_speed = Units.feetToMeters(max_speed_feet);
    max_angular_speed = max_drive_speed / drivetrain_radius;
  }

  @AutoLogOutput(key = "Swerve/AutoAimingAngle")
  public double getAutoAimingAngle() {
    Pose2d robotPose = getFusedOdometry();
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d speakerVector = new Translation2d();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerVector = robotTranslation.minus(new Translation2d(0, Constants.FieldConstants.blueSpeakerReferencePoint.getY()));
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      speakerVector = robotTranslation.minus(new Translation2d(Constants.FieldConstants.redSpeakerReferencePoint.getX(), Constants.FieldConstants.redSpeakerReferencePoint.getY()));
    }

    return speakerVector.getAngle().getRadians();
  }

  public double getHeadingFeedback(Rotation2d setPoint) {
    headingFeedbackVal = headingPID.calculate(getFusedOdometry().getRotation().getRadians(), setPoint.getRadians());
    return headingPID.calculate(getFusedOdometry().getRotation().getRadians(), setPoint.getRadians());
  }

  public double getMaxLinearSpeed() {    
    return max_drive_speed;
  }

  public double getMaxAngularSpeed() {
    return max_angular_speed;
  }

  // Stop robot
  public void stop() {
    runVelc(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // Aigns all modules to the front and basically acts like a tank drive
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelc();
    }
    return driveVelocityAverage / 4.0;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  // Return robot position
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }


  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  // Get rotation
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePos(), pose);
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(track_width_x / 2.0, track_width_y / 2.0),
      new Translation2d(track_width_x / 2.0, -track_width_y / 2.0),
      new Translation2d(-track_width_x / 2.0, track_width_y / 2.0),
      new Translation2d(-track_width_x / 2.0, -track_width_y / 2.0)
    };
  }
}
