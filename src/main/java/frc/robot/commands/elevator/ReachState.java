// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ReachState extends Command {
  /** Creates a new SetElevatorLength. */
  boolean end = false;
  boolean thresholdEnabled;
  double thresholdAngle;
  String stateName;

  public ReachState(String stateName, boolean thresholdEnabled, double thresholdAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stateName = stateName;
    this.thresholdEnabled = thresholdEnabled;
    this.thresholdAngle = thresholdAngle;
    addRequirements(RobotContainer.m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_elevator.reachState(stateName);
    if (thresholdEnabled && RobotContainer.m_elevator.getAngleDeg() < thresholdAngle) {
      end = true;
    } else if (thresholdEnabled == false) {
      end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
