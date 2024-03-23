// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveControl extends Command {
  Swerve swerve;
  DoubleSupplier xSupp, ySupp, omegaSupp;
  boolean autoAiming;
  private static final double DEADBAND = 0.1;

  /** Creates a new SwerveControl. */
  public SwerveControl(Swerve swerve, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier omegaSupp, boolean autoAiming) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
    this.swerve = swerve;
    this.xSupp = xSupp;
    this.ySupp = ySupp;
    this.omegaSupp = omegaSupp;
    this.autoAiming = autoAiming;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupp.getAsDouble(), ySupp.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupp.getAsDouble(), ySupp.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupp.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

    Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          RobotContainer.m_swerve.runVelc(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * RobotContainer.m_swerve.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * RobotContainer.m_swerve.getMaxLinearSpeedMetersPerSec(),
                  omega * RobotContainer.m_swerve.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? RobotContainer.m_swerve.getRotation().plus(new Rotation2d(Math.PI))
                      : RobotContainer.m_swerve.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
