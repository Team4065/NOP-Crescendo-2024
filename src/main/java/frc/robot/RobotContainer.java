// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.ResetOdo;
import frc.robot.commands.SwerveControl;
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

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static Swerve m_swerve;
  public static Vision m_vision;

  public static GenericHID controller = new GenericHID(0);
  public static JoystickButton AB = new JoystickButton(controller, 1);
  public static JoystickButton XB = new JoystickButton(controller, 3);
  public static JoystickButton BB = new JoystickButton(controller, 2);

  static LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Chooser");
  static HashMap<Command, String> autoMap = new HashMap<>();


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
        break;
      case SIM:
        m_swerve = new Swerve(
          new GyroIO() {},
          new ModuleIOSim(),
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim()
        );

        m_vision = new Vision(new VisionSimIO());

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
        break;
    }

    m_chooser.addDefaultOption("P1 - 3R", AutoCommandBuilder.returnAutoCommand("Pos 1- 3 rings"));
    Shuffleboard.getTab("Autonomous").add(m_chooser.getSendableChooser()).withSize(3, 1);

    // Setting up PathPlanner auto selector
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    m_swerve.setDefaultCommand(new SwerveControl(
      m_swerve, 
      () -> -controller.getRawAxis(1), 
      () -> -controller.getRawAxis(0), 
      () -> -controller.getRawAxis(4)
    ));

    XB.onTrue(new ResetOdo());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.get();
  }
}
