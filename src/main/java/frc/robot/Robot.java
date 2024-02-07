// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LocalADStarAK;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.pathfinding.Pathfinding;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static String allianceColor;
  ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  GenericEntry allianceColorWidget = autoTab.add("ALLIANCE", true) 
    .withProperties(Map.of("colorWhenTrue", "blue"))
    .withPosition(0, 1)
    .withSize(3, 1)
    .getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    switch (Constants.currentMode) {
      // Running on a real robot, log to a USB stick 
      // The "/U" is there to indicate that the roboRIO will store on the USB
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

        // Running a physics simulator, log to local folder
      case SIM:
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

        // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = "";
        Logger.setReplaySource(new WPILOGReader(logPath));
        break;
    }

    Logger.start();
    m_robotContainer = new RobotContainer();

    Shuffleboard.getTab("Autonomous").add(Constants.displayField);

    // Set A* algorithim for AdvantageKit as the default path-finding algorithim
    Pathfinding.setPathfinder(new LocalADStarAK());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    // red --> 1, 2, 3
    // blue --> 4, 5, 6
    int robotPos = LoggedDriverStation.getDSData().allianceStation; 
    allianceColor = (robotPos > 3) ? "BLUE" : "RED";

    if (robotPos > 3) {
      allianceColorWidget.setBoolean(true);
    } else if (robotPos <= 3) {
      allianceColorWidget.setBoolean(false);
    } else {
      allianceColorWidget.setString("ERROR");
    }

    Trajectory finalTrajectoryToDisplay;

    if (RobotContainer.m_chooser.get() != RobotContainer.noAutoCommand) {
      int pathsInAuto = PathPlannerAuto.getPathGroupFromAutoFile(Constants.autoRoutines.get(RobotContainer.m_chooser.get())).size();
      List<Pose2d> posesOnTrajectory = new ArrayList<>();

     

      for (int i = 0; i < pathsInAuto; i++) {
          PathPlannerPath currentPath = PathPlannerAuto.getPathGroupFromAutoFile(Constants.autoRoutines.get(RobotContainer.m_chooser.get())).get(i);
          List<PathPoint> pathPoints = currentPath.getAllPathPoints();
        for (int j = 0; j < pathPoints.size(); j++) {
          posesOnTrajectory.add(j, new Pose2d(pathPoints.get(j).position, new Rotation2d(0)));
        }
      }

      

      finalTrajectoryToDisplay = TrajectoryGenerator.generateTrajectory(posesOnTrajectory, new TrajectoryConfig(Units.metersToFeet(Constants.SwerveConstants.MAX_SPEED_FEET), Units.metersToFeet(Constants.SwerveConstants.MAX_SPEED_FEET)));

    } else {
      finalTrajectoryToDisplay = new Trajectory();
    }

    Constants.displayField.getObject("Field").setTrajectory(finalTrajectoryToDisplay);

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
