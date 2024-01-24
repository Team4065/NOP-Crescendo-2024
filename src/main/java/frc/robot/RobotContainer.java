// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.ResetOdo;
import frc.robot.commands.SetElevatorLength;
import frc.robot.commands.SwerveControl;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.limelight.Vision;
import frc.robot.subsystems.limelight.VisionIO;
import frc.robot.subsystems.limelight.VisionLimelight;
import frc.robot.subsystems.limelight.VisionSimIO;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.modules.ModuleIO;
import frc.robot.subsystems.swerve.modules.ModuleIOSim;
import frc.robot.subsystems.swerve.modules.ModuleIOTalonFX;
import frc.robot.util.AutoCommandBuilder;
import frc.robot.util.PathFindingWithPath;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {
  public static Swerve m_swerve;
  public static Vision m_vision;
  public static Elevator m_elevator;

  public static GenericHID controller = new GenericHID(0);
  public static JoystickButton AB = new JoystickButton(controller, 1);
  public static JoystickButton XB = new JoystickButton(controller, 3);
  public static JoystickButton BB = new JoystickButton(controller, 2);
  public static JoystickButton YB = new JoystickButton(controller, 4);

  public static POVButton upButton = new POVButton(controller, 0);
  public static POVButton downButton = new POVButton(controller, 180);
  public static POVButton rightButton = new POVButton(controller, 90);
  public static POVButton leftButton = new POVButton(controller, 270);

  public static LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Chooser");

  public static Command noAutoCommand = new InstantCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_swerve = new Swerve(
          new GyroIONavX(),
          new ModuleIOTalonFX(0),
          new ModuleIOTalonFX(1),
          new ModuleIOTalonFX(2),
          new ModuleIOTalonFX(3)
        );
        m_vision = new Vision(new VisionLimelight("limelight-nop"));
        m_elevator = new Elevator(new ElevatorIO() {});

        break;
      case SIM:
        m_swerve = new Swerve(
          new GyroIO() {},
          new ModuleIOSim(),
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim()
        );
        m_elevator = new Elevator(new ElevatorIOSim());
        m_vision = new Vision(new VisionSimIO(m_swerve::getPose));

        break;
      default:
        m_swerve = new Swerve(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );
        m_vision = new Vision(new VisionIO() {});
        m_elevator = new Elevator(new ElevatorIO() {});

        break;
    }

    m_chooser.addDefaultOption("NOTHING", noAutoCommand);
    m_chooser.addOption("P1 - 3R", AutoCommandBuilder.returnAutoCommand("Pos 1 - 3 rings"));
    Shuffleboard.getTab("Autonomous").add(m_chooser.getSendableChooser()).withSize(3, 1);

    configureBindings();
  }

  private void configureBindings() {
    m_swerve.setDefaultCommand(new SwerveControl(
      m_swerve, 
      () -> -controller.getRawAxis(1), 
      () -> -controller.getRawAxis(0), 
      () -> -controller.getRawAxis(4)
    ));

    AB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("Stage Middle Finisher", AB));
    BB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("Source Finisher 1", BB));
    XB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("AMP Finisher", XB));

    YB.onTrue(new ResetOdo());

    upButton.onTrue(new SetElevatorLength(4));
    downButton.onTrue(new SetElevatorLength(2));
  }

  public Command getAutonomousCommand() {
    return m_chooser.get();
  }
}
