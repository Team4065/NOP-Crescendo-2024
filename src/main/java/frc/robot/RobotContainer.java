// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.climber.RaiseClimber;
import frc.robot.commands.elevator.ReachState;
import frc.robot.commands.shooter.SetIntakeSpeed;
import frc.robot.commands.shooter.SetShooterSpeed;
import frc.robot.commands.swerve.ResetOdo;
import frc.robot.commands.swerve.SwerveControl;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleIO;
import frc.robot.subsystems.swerve.modules.ModuleIOSim;
import frc.robot.subsystems.swerve.modules.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionSimIO;
import frc.robot.util.AutoCommandBuilder;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.PathFindingWithPath;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {
  public static Swerve m_swerve;
  public static Vision m_vision;
  public static Elevator m_elevator;
  public static Shooter m_shooter;
  public static Climber m_climber;

  public static PowerDistribution pdh;

  public static GenericHID controller = new GenericHID(0);
  public static JoystickButton AB = new JoystickButton(controller, 1);
  public static JoystickButton XB = new JoystickButton(controller, 3);
  public static JoystickButton BB = new JoystickButton(controller, 2);
  public static JoystickButton YB = new JoystickButton(controller, 4);
  public static JoystickButton leftBumper = new JoystickButton(controller, 5);
  public static JoystickButton rightBumper = new JoystickButton(controller, 6);

  public static POVButton upButton = new POVButton(controller, 0);
  public static POVButton downButton = new POVButton(controller, 180);
  public static POVButton rightButton = new POVButton(controller, 90);
  public static POVButton leftButton = new POVButton(controller, 270);

  public static LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Chooser");

  public static Command noAutoCommand = new InstantCommand();

  public static NoteVisualizer noteVis = new NoteVisualizer();

  


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

        m_vision = new Vision(
          new VisionIO() {},
          new VisionIO() {}
        );

        m_shooter = new Shooter(
          new ShooterIOReal()
        );

        m_elevator = new Elevator(
          new ElevatorIOTalonFX()
        );

        m_climber = new Climber(
          new ClimberIOReal()
        );

        pdh = new PowerDistribution(Constants.pdhCANID, ModuleType.kRev);
        pdh.setSwitchableChannel(true);
        
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

        m_vision = new Vision(
          new VisionSimIO(
            m_swerve::getPose,
            Constants.LimelightPositions.camPosBL,
            "sim_cam_BL"
          ),
          new VisionSimIO(
            m_swerve::getPose,
            Constants.LimelightPositions.camPosBR,
            "sim_cam_BR"
          )
        );

        m_shooter = new Shooter(new ShooterIO() {});
        
        m_climber = new Climber(new ClimberIO() {});

        break;
      default:
        m_swerve = new Swerve(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );

        m_vision = new Vision(
          new VisionIO() {},
          new VisionIO() {}
        );

        m_elevator = new Elevator(new ElevatorIO() {});

        m_shooter = new Shooter(new ShooterIO() {});
        
        m_climber = new Climber(new ClimberIO() {});


        break;
    }

    NoteVisualizer.setRobotPoseSupplier(() -> RobotContainer.m_swerve.getPose());

    NamedCommands.registerCommand("deploy", new ReachState("intake"));
    NamedCommands.registerCommand("retract", new ReachState("in"));

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
    
    // rightBumper.whileTrue(new SwerveControl(
    //   m_swerve, 
    //   () -> -controller.getRawAxis(1), 
    //   () -> -controller.getRawAxis(0), 
    //   () -> MathUtil.clamp(m_swerve.getHeadingFeedback(new Rotation2d(m_swerve.getAutoAimingAngle())), -1, 1)
    // ));

    /* AB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("Stage Middle Finisher", AB));
    BB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("Source Finisher 1", BB));
    XB.onTrue(PathFindingWithPath.pathFindingAutoBuilder("AMP Finisher", XB)); */

    // Manual Control
    XB.onTrue(new InstantCommand(() -> {m_elevator.setTiltVoltage(-1);}));
    BB.onTrue(new InstantCommand(() -> {m_elevator.setTiltVoltage(0);}));

    YB.onTrue(new InstantCommand(() -> {m_elevator.setExtensionVoltage(1);}));
    AB.onTrue(new InstantCommand(() -> {m_elevator.setExtensionVoltage(-1);}));
    leftBumper.onTrue(new InstantCommand(() -> {m_elevator.setExtensionVoltage(0);}));

    rightBumper.onTrue(new ResetOdo());

    upButton.onTrue(new SetShooterSpeed(8));
    downButton.onTrue(new SetShooterSpeed(0));

    leftButton.onTrue(new SetIntakeSpeed(8));
    rightButton.onTrue(new SetIntakeSpeed(0));
   
    

    // YB.whileTrue(m_elevator.routine.quasistatic(Direction.kForward));
    // AB.whileTrue(m_elevator.routine.quasistatic(Direction.kReverse));
    
    // BB.whileTrue(m_elevator.routine.dynamic(Direction.kForward));
    // XB.whileTrue(m_elevator.routine.dynamic(Direction.kReverse));

    // XB.onTrue(new InstantCommand(() -> {m_elevator.reachExtension(0);}));
    // YB.onTrue(new InstantCommand(() -> {m_elevator.reachExtension(8);}));

  }

  public Command getAutonomousCommand() {
    return m_chooser.get();
  }
}