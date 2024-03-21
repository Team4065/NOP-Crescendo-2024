// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber.left;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class LeftRatchet extends Command {
  /** Creates a new LeftRatchet. */
  boolean end, state;
  public LeftRatchet(boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_climber.setLeftRatchet(state);
    end = true;
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
