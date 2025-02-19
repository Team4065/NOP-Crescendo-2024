// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoTilt extends Command {
  /** Creates a new AutoTikt. */
  double angle;
  public AutoTilt() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("distance: " + Units.metersToInches(RobotContainer.m_swerve.getDistanceFromSpeaker("blue")));
    angle = Constants.ElevatorConstants.map.get(Units.metersToInches(RobotContainer.m_swerve.getDistanceFromSpeaker("blue")));
    RobotContainer.m_elevator.setAngle(angle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(RobotContainer.AB.getAsBoolean());
  }
}
