package frc.robot.subsystems.limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limelight.Vision.SnapshotMode;

public class VisionSimIO implements VisionIO {
  // Using Photonlib to "simulate" limelight vision
  String simCamName;

  // Set up virtual april tag field
  AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  VisionSystemSim visionFieldSim;

   // Set up camera sim
  SimCameraProperties cameraProp = new SimCameraProperties();

  PhotonCamera fakeCamera;

  PhotonCameraSim cameraSim;


  // Transform3d cameraPosToRobot = new Transform3d(new Translation3d(0.025, 0.3, 0), new Rotation3d(0, 0, 0));
  Transform3d cameraPosToRobot;

  PhotonPoseEstimator photonPoseEstimator; 

  Supplier<Pose2d> currentRobotPoseSupp;

  public VisionSimIO(Supplier<Pose2d> currentRobotPoseSupp, Transform3d cameraPosToRobot, String simCamName) {
    visionFieldSim = new VisionSystemSim("main");
    visionFieldSim.addAprilTags(tagLayout);

    this.cameraPosToRobot = cameraPosToRobot;
    this.simCamName = simCamName;

    fakeCamera = new PhotonCamera(simCamName);
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, fakeCamera, cameraPosToRobot); 
    
    cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(60));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(40);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(fakeCamera, cameraProp);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableRawStream(true);
    cameraSim.enableDrawWireframe(true);
    

    visionFieldSim.addCamera(cameraSim, cameraPosToRobot);

    this.currentRobotPoseSupp = currentRobotPoseSupp;
  }

  public String getName() {
    return simCamName;
  }


  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return photonPoseEstimator.update();
  }

  public static PhotonPipelineResult results;

  @Override
  public void updateInputs(VisionInputs inputs) {
    // TO-DO update sim field pose of the robot
    // Implement necessary inputs
  
    results = cameraSim.getCamera().getLatestResult();

    visionFieldSim.update(currentRobotPoseSupp.get());
    var estimatedPhotonPose = getEstimatedGlobalPose();
    Pose2d estimatedVisionPose;

    if (estimatedPhotonPose.isEmpty()) {
      estimatedVisionPose = new Pose2d();
    } else {
      estimatedVisionPose = estimatedPhotonPose.get().estimatedPose.toPose2d();
    }
    

    inputs.validTarget = results.hasTargets();
    inputs.horizontalCrosshairOffset = 0;
    inputs.verticalCrosshairOffset = 0;
    inputs.targetArea = results.hasTargets() ? results.getBestTarget().getArea() : 0;
    inputs.skew = results.hasTargets() ? results.getBestTarget().getSkew() : 0;
    inputs.pipelineLatency = results.getLatencyMillis();
    inputs.shortSidelength = 0;
    inputs.longSideLength = 0;
    inputs.horizontalSideLength = 0;
    inputs.verticalSideLength = 0;
    inputs.pipelineIndex = 0;
    inputs.jsonDump = " ";
    inputs.botpose_wpiblue = new double[] {estimatedVisionPose.getX(), estimatedVisionPose.getY(), 0, 0, 0, estimatedVisionPose.getRotation().getDegrees(), results.getLatencyMillis()};
    inputs.botpose_wpired = new double[] {estimatedVisionPose.getX(), estimatedVisionPose.getY(), 0, 0, 0, estimatedVisionPose.getRotation().getDegrees(), results.getLatencyMillis()};
    inputs.camerapose_targetspace = new double[] {};
    inputs.targetpose_cameraspace = new double[] {};
    inputs.targetpose_robotspace = new double[] {};
    inputs.botpose_targetspace = new double[] {};
    inputs.camerapose_robotspace = new double[] {};
    inputs.botpose = new double[] {estimatedVisionPose.getX(), estimatedVisionPose.getY(), 0, 0, 0, estimatedVisionPose.getRotation().getDegrees(), results.getLatencyMillis()};
    inputs.tid = results.hasTargets() ? results.getBestTarget().getFiducialId() : 0;
    inputs.neuralDetectorID = 0;
    inputs.ledMode = 0;
    inputs.camMode = 0;
    inputs.pipeline = 0;
    inputs.stream = 0;
    inputs.snapshot = false; // IO layer
    inputs.crop = new double[] {};
    inputs.numTargets = 0;
  }

  @Override
  public void setSnapshotMode(SnapshotMode mode) {}
}
