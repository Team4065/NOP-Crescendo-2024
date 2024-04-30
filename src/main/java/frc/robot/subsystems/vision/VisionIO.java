package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.IndividualCam.SnapshotMode;

public interface VisionIO {
  @AutoLog
  public static class VisionInputs {
    public boolean validTarget = false;
    public double horizontalCrosshairOffset = 0;
    public double verticalCrosshairOffset = 0;
    public double targetArea = 0;
    public double skew = 0;
    public double pipelineLatency = 0;
    public double shortSidelength = 0;
    public double longSideLength = 0;
    public double horizontalSideLength = 0;
    public double verticalSideLength = 0;
    public long pipelineIndex = 0;
    public String jsonDump = " ";
    public double[] botpose = new double[] {};
    public double[] botpose_wpiblue = new double[] {};
    public double[] botpose_wpired = new double[] {};
    public double[] camerapose_targetspace = new double[] {};
    public double[] targetpose_cameraspace = new double[] {};
    public double[] targetpose_robotspace = new double[] {};
    public double[] botpose_targetspace = new double[] {};
    public double[] camerapose_robotspace = new double[] {};
    public long tid = 0;
    public long neuralDetectorID = 0;
    public long ledMode = 0;
    public long camMode = 0;
    public long pipeline = 0;
    public long stream = 0;
    public boolean snapshot = false; // IO layer
    public double[] crop = new double[] {};
    public double numTargets = 0;
    public Pose2d mt2 = new Pose2d();
  }
    

  public default String getName() { return new String(); }

  public default void updateInputs(VisionInputs inputs) {}

  public default void setSnapshotMode(SnapshotMode mode) {}
}
