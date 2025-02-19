// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetIntakeSpeed extends Command {
  /** Creates a new SetIntakeSpeed. */
  boolean end;
  double speed;

  public SetIntakeSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
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
    if (RobotContainer.m_shooter.getBeamBreak()) {
      RobotContainer.m_shooter.setIntakeVoltage(speed);
    }
    end = !RobotContainer.m_shooter.getBeamBreak();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
