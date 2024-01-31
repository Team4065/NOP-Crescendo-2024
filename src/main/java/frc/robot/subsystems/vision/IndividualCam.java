// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IndividualCam extends SubsystemBase {
  private boolean dashboardSignal = false;
  private double limelightHeightInches;
  private VisionIO camera;
  // private VisionInputsAutoLogged cameraInputs;
  private VisionInputsAutoLogged cameraInputs;

  int index;


  public IndividualCam(VisionIO camera, int index) {
    cameraInputs = new VisionInputsAutoLogged();

    camera.updateInputs(cameraInputs);
    this.camera = camera;
    this.index = index;
  }


  @Override
  public void periodic() {
    camera.updateInputs(cameraInputs);
    Logger.processInputs("Vision/Camera" + Integer.toString(index), cameraInputs);
    Logger.recordOutput("Vision/Cameras" + Integer.toString(index) + "/PoseEstimation", getCameraPose());

    List<Pose3d> detectedTagPoses = getTagPoses();

    Logger.recordOutput("Vision/Cameras" + Integer.toString(index) + "/TagPoses", detectedTagPoses.toArray(new Pose3d[detectedTagPoses.size()]));
  }

  public List<Pose3d> getTagPoses() {
    List<Pose3d> detectedTagPoses = new ArrayList<>();
    for (int i = 0; i < VisionSimIO.results.getTargets().size(); i++) {
      detectedTagPoses.add(Constants.tagLayout.getTagPose(VisionSimIO.results.getTargets().get(i).getFiducialId()).get());
    }

    return detectedTagPoses;
  }

  private NetworkTable getTable(String table) {
    return NetworkTableInstance.getDefault().getTable(table);
  }

  private NetworkTableEntry getEntry(String entryName, String table) {
    return getTable(table).getEntry(entryName);
  }

  private double getValue(String entryName, String table) {
    return getEntry(entryName, table).getDouble(0);
  }

  private void setValue(String entryName, double value, String table) {
    getEntry(entryName, table).setNumber(value);
  }

  public VisionInputsAutoLogged getCameraInputs() {
    return cameraInputs;
  }

  public enum LedMode {
    PIPELINE(0),
    FORCE_OFF(1),
    FORCE_BLINK(2),
    FORCE_ON(3),
    UNKNOWN(-1);

    public double value;

    LedMode(double value) {
      this.value = value;
    }
  }

  public enum Limelights {
    FRONT("limelight-nop");

    public String table;

    Limelights(String table) {
      this.table = table;
    }
  }

  /**
   * @return The current LED mode set on the Limelight
   */
  public LedMode getLedMode(Limelights limelight) {
    double mode = getValue("ledMode", limelight.table);
    if (mode == 0) {
      return LedMode.PIPELINE; // Uses the LED mode set in the pipeliine
    } else if (mode == 1) {
      return LedMode.FORCE_OFF;
    } else if (mode == 2) {
      return LedMode.FORCE_BLINK;
    } else if (mode == 3) {
      return LedMode.FORCE_ON;
    } else {
      System.out.println("[Limelight] UNKNOWN LED MODE -- " + mode);
      return LedMode.UNKNOWN;
    }
  }

  public void setLedMode(LedMode mode, Limelights limelight) {
    if (mode != LedMode.UNKNOWN) {
      setValue("ledMode", mode.value, limelight.table);
    }
  }

  public double getDistanceFromGoal() {
    double targetOffsetAngle_Vertical = cameraInputs.verticalCrosshairOffset;

    // how many degrees back is your limelight rotated from perfectly vertical?
    // CHECK
    double limelightMountAngleDegrees = 0;

    // distance from the center of the Limelight lens to the floor
    // CHANGE how high up it is
    double limelightLensHeightInches = 12.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    return (goalHeightInches - limelightHeightInches) / Math.tan(angleToGoalRadians);
  }

  public enum Pipeline {
    PIPELINE0(0),
    PIPELINE1(1),
    UNKNOWN(-1);

    public double value;

    Pipeline(double value) {
      this.value = value;
    }
  }

  public Pipeline getCurrentPipeline(VisionInputsAutoLogged inputs) {
    long mode = inputs.pipeline;

    if (mode == 0) {
      return Pipeline.PIPELINE0;
    } else if (mode == 1) {
      return Pipeline.PIPELINE1;
    } else {
      System.out.println("[Limelight] UNKNOWN Pipeline -- " + mode);
      return Pipeline.UNKNOWN;
    }
  }

  
  /**
   * @param mode The LED Mode to set on the Limelight
   */
  public void setPipeline(Pipeline mode, Limelights limelight) {
    if (mode != Pipeline.UNKNOWN) {
      setValue("pipeline", mode.value, limelight.table);
    }
  }

  public enum StreamMode {
    STANDARD(0),
    PIP_MAIN(1),
    PIP_SECONDARY(2),
    UNKNOWN(-1);

    public double value;

    StreamMode(double value) {
      this.value = value;
    }
  }

  /**
   * @return The current LED mode set on the Limelight
   */
  public StreamMode getCurrentStreamMode(VisionInputsAutoLogged inputs) {
    double mode = inputs.stream;
    if (mode == 0) {
      return StreamMode.STANDARD; // Side-by-side streams if a webcam is attached to Limelight
    } else if (mode == 1) {
      return StreamMode
          .PIP_MAIN; // The secondary camera stream is placed in the lower-right corner of the
      // primary camera stream
    } else if (mode == 2) {
      return StreamMode.PIP_SECONDARY;
    } else {
      System.out.println("[Limelight] UNKNOWN StreamMode -- " + mode);
      return StreamMode.UNKNOWN;
    }
  }

  /**
   * @param mode The LED Mode to set on the Limelight
   */
  public void setStreamMode(StreamMode mode, Limelights limelight) {
    if (mode != StreamMode.UNKNOWN) {
      setValue("stream", mode.value, limelight.table);
    }
  }

  public enum SnapshotMode {
    OFF(0),
    TWO_PER_SECOND(1),
    UNKNOWN(-1);

    public double value;

    SnapshotMode(double value) {
      this.value = value;
    }
  }

  public void setCurrentSnapshotMode(SnapshotMode mode) {
    camera.setSnapshotMode(mode);
  }

  /**
   * @return The current LED mode set on the Limelight
   */
  public SnapshotMode getCurrentSnapShotMode() {
    boolean mode = cameraInputs.snapshot;
    if (mode == false) {
      return SnapshotMode.OFF;
    } else if (mode == true) {
      return SnapshotMode.TWO_PER_SECOND;
    } else {
      System.out.println("[Limelight] UNKNOWN SnapshotMode -- " + mode);
      return SnapshotMode.UNKNOWN;
    }
  }

  public Pose2d getCameraPose() {
    if (cameraInputs.botpose_wpiblue.length > 0) {
      return new Pose2d(cameraInputs.botpose_wpiblue[0], cameraInputs.botpose_wpiblue[1], new Rotation2d(Units.degreesToRadians(cameraInputs.botpose_wpired[5])));
    } else {
      return new Pose2d();
    }
  }

  public boolean hasMultipleTargets(Limelights limelight) {
    return (limelight == Limelights.FRONT ? cameraInputs.numTargets : cameraInputs.numTargets) > 1;
  }
}