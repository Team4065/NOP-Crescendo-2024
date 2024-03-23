// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetShooterSpeed extends Command {
  /** Creates a new SetIntakeSpeed. */
  boolean end;
  double threshold;
  boolean thresholdEnabled;
  double speed;

  public SetShooterSpeed(double speed, boolean thresholdEnabled, double threshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    this.threshold = threshold;
    this.thresholdEnabled = thresholdEnabled;

    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_shooter.setShooterVoltage(speed);

    if (RobotContainer.m_shooter.getShooterVelc() > threshold) {
      RobotContainer.m_shooter.setIntakeVoltage(speed);
    }

    if (thresholdEnabled) {
      // end = (RobotContainer.m_shooter.getShooterVelc() > threshold);
      end = (RobotContainer.m_shooter.getShooterVelc()) > threshold && (RobotContainer.m_shooter.getBeamBreak());
    } else if (thresholdEnabled == false) {
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
