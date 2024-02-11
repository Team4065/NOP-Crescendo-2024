// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  IndividualCam[] cameras = new IndividualCam[2];

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    RobotContainer.m_swerve.getKinematics(), 
    RobotContainer.m_swerve.getRotation(), 
    RobotContainer.m_swerve.getModulePos(), 
    new Pose2d(), 
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
  );

  public Vision(VisionIO cameraBL, VisionIO cameraBR) {
    cameras[0] = new IndividualCam(cameraBL, 2);
    cameras[1] = new IndividualCam(cameraBR, 3);
  }

  @Override
  public void periodic() {
    poseEstimator.update(RobotContainer.m_swerve.getRotation(), RobotContainer.m_swerve.getModulePos());

    for (var camera : cameras) {
      if (camera.getCameraInputs().botpose.length > 0) {
        poseEstimator.addVisionMeasurement(camera.getCameraPose(), Timer.getFPGATimestamp() - (camera.getCameraInputs().botpose_wpired[6] / 1000.0));
      }
    }
  }

  @AutoLogOutput(key = "Vision/EstimatedPose2d")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose3d[] cameraPoses() {
    Pose3d[] posesToReturn = new Pose3d[4];

    posesToReturn[0] = new Pose3d(
      new Translation3d(Constants.LimelightPositions.camPosBR.getX(), Constants.LimelightPositions.camPosBR.getY(), Constants.LimelightPositions.camPosBR.getZ()), 
      new Rotation3d(Constants.LimelightPositions.camPosBR.getRotation().getX(), Constants.LimelightPositions.camPosBR.getRotation().getY(), Constants.LimelightPositions.camPosBR.getRotation().getZ())
    );
    posesToReturn[1] = new Pose3d(
      new Translation3d(Constants.LimelightPositions.camPosBL.getX(), Constants.LimelightPositions.camPosBL.getY(), Constants.LimelightPositions.camPosBL.getZ()), 
      new Rotation3d(Constants.LimelightPositions.camPosBL.getRotation().getX(), Constants.LimelightPositions.camPosBL.getRotation().getY(), Constants.LimelightPositions.camPosBL.getRotation().getZ())
    );
    posesToReturn[2] = new Pose3d(
      new Translation3d(Constants.LimelightPositions.camPosNoteCamR.getX(), Constants.LimelightPositions.camPosNoteCamR.getY(), Constants.LimelightPositions.camPosNoteCamR.getZ()), 
      new Rotation3d(Constants.LimelightPositions.camPosNoteCamR.getRotation().getX(), Constants.LimelightPositions.camPosNoteCamR.getRotation().getY(), Constants.LimelightPositions.camPosNoteCamR.getRotation().getZ())
    );
    posesToReturn[3] = new Pose3d(
      new Translation3d(Constants.LimelightPositions.camPosNoteCamL.getX(), Constants.LimelightPositions.camPosNoteCamL.getY(), Constants.LimelightPositions.camPosNoteCamL.getZ()), 
      new Rotation3d(Constants.LimelightPositions.camPosNoteCamL.getRotation().getX(), Constants.LimelightPositions.camPosNoteCamL.getRotation().getY(), Constants.LimelightPositions.camPosNoteCamL.getRotation().getZ())
    );

    return posesToReturn;
  }
}
