// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
