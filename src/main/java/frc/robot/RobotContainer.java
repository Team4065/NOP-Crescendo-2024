// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Volts;

import java.time.Instant;
import java.util.function.Consumer;

import javax.swing.plaf.TextUI;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.climber.both.ActivateRatchet;
import frc.robot.commands.climber.both.RaiseClimber;
import frc.robot.commands.climber.left.LeftMotor;
import frc.robot.commands.climber.left.LeftRatchet;
import frc.robot.commands.climber.right.RightMotor;
import frc.robot.commands.climber.right.RightRatchet;
import frc.robot.commands.elevator.ReachCustomState;
import frc.robot.commands.elevator.ReachState;
import frc.robot.commands.shooter.SetIntakeSpeed;
import frc.robot.commands.shooter.SetShooterSpeed;
import frc.robot.commands.swerve.ResetOdo;
import frc.robot.commands.swerve.SetSpeed;
import frc.robot.commands.swerve.SwerveControl;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
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
import frc.robot.subsystems.vision.VisionLimelight;
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
  public static LEDs m_leds;

  public static PowerDistribution pdh;

  public static XboxController controller = new XboxController(0);
  public static JoystickButton AB = new JoystickButton(controller, 1);
  public static JoystickButton XB = new JoystickButton(controller, 3);
  public static JoystickButton BB = new JoystickButton(controller, 2);
  public static JoystickButton YB = new JoystickButton(controller, 4);
  public static JoystickButton leftBumper = new JoystickButton(controller, 5);
  public static JoystickButton rightBumper = new JoystickButton(controller, 6);
  public static JoystickButton windowButton = new JoystickButton(controller, 7); // reset gyro

  public static POVButton upButton = new POVButton(controller, 0);
  public static POVButton downButton = new POVButton(controller, 180);
  public static POVButton rightButton = new POVButton(controller, 90);
  public static POVButton leftButton = new POVButton(controller, 270);

  public static Joystick buttonBox = new Joystick(1);
  public static JoystickButton B4 = new JoystickButton(buttonBox, 10);
  public static JoystickButton B5 = new JoystickButton(buttonBox, 9);
  public static JoystickButton B6 = new JoystickButton(buttonBox, 11);
  public static JoystickButton B7 = new JoystickButton(buttonBox, 12);
  public static JoystickButton B1 = new JoystickButton(buttonBox, 6);
  public static JoystickButton B2 = new JoystickButton(buttonBox, 8);
  public static JoystickButton B3 = new JoystickButton(buttonBox, 7);

  public static LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Chooser");

  public static Command noAutoCommand = new ResetOdo();

  public static NoteVisualizer noteVis = new NoteVisualizer();

  // UPLOAD CODE ON 4/6/2024

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

        // m_vision = new Vision(
        //   new VisionLimelight("limelight-nopfl"),
        //   new VisionLimelight("limelight-nopfr"),
        //   new VisionLimelight("limelight-nopbl"),
        //   new VisionLimelight("limelight-nopbr")
        // );

        m_vision = new Vision(
          new VisionIO() {},
          new VisionIO() {},
          new VisionLimelight("limelight-nopbl") {},
          new VisionLimelight("limelight-nopbr") {}
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

        m_leds = new LEDs();

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
          new VisionIO() {},
          new VisionIO() {},
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

        m_leds = new LEDs();

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
          new VisionIO() {},
          new VisionIO() {},
          new VisionIO() {}
        );

        m_elevator = new Elevator(new ElevatorIO() {});

        m_shooter = new Shooter(new ShooterIO() {});
        
        m_climber = new Climber(new ClimberIO() {});


        break;
    }

    NoteVisualizer.setRobotPoseSupplier(() -> RobotContainer.m_swerve.getPose());
    if (Constants.currentMode == Mode.REAL) {
      NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_leds.setState("rainbow");
        }),
        new SetShooterSpeed(9, true, 72)
      ));
      NamedCommands.registerCommand("stop", new SequentialCommandGroup(new SetShooterSpeed(0, false, 0), new InstantCommand(() -> {m_shooter.setIntakeVoltage(0);m_leds.setState("idle");})));
      NamedCommands.registerCommand("deploy", new SequentialCommandGroup(new ReachState("intake", false, 0), new SetIntakeSpeed(5)));
      NamedCommands.registerCommand("retract", new ReachState("in", true, 12.5));
      NamedCommands.registerCommand("autoTilt", new ReachCustomState(27, true, 0));
      NamedCommands.registerCommand("autoTilt 2", new ReachCustomState(34, true, 0));
      NamedCommands.registerCommand("back", new SequentialCommandGroup(
        new InstantCommand(() -> {m_shooter.setIntakeVoltage(-1.5);}),
        new WaitCommand(0.2),
        new InstantCommand(() -> {m_shooter.setIntakeVoltage(0);})
      ));
    } else {
      NamedCommands.registerCommand("shoot", new InstantCommand());
      NamedCommands.registerCommand("stop", new InstantCommand());
      NamedCommands.registerCommand("deploy", new InstantCommand());
      NamedCommands.registerCommand("retract", new InstantCommand());
      NamedCommands.registerCommand("autoTilt", new InstantCommand());
    }
    

    m_chooser.addDefaultOption("NOTHING", noAutoCommand);

    m_chooser.addOption("P1 - 1R", AutoCommandBuilder.returnAutoCommand("Pos 1 - 1 ring"));
    m_chooser.addOption("P2 - 2R", AutoCommandBuilder.returnAutoCommand("Pos 2 - 2 Rings"));
    m_chooser.addOption("P3 - 1R", AutoCommandBuilder.returnAutoCommand("Pos 3 - 1 ring"));


    Shuffleboard.getTab("Autonomous").add(m_chooser.getSendableChooser()).withSize(3, 1);

    configureBindings();
  }

  private void configureBindings() {
    m_swerve.setDefaultCommand(new SwerveControl(
      m_swerve, 
      () -> {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
          return -controller.getRawAxis(1);
        } else {
          return -controller.getRawAxis(1);
        }
      }, 
      () -> {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
          return -controller.getRawAxis(0);
        } else {
          return -controller.getRawAxis(0);
        }
      }, 
      () -> -controller.getRawAxis(4), 
      false
    ));
    
    // AB.whileTrue(new SequentialCommandGroup(
    //   new SetSpeed(5),
    //   new SwerveControl(
    //     m_swerve, 
    //     () -> {
    //       if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         return -controller.getRawAxis(1);
    //       } else {
    //         return controller.getRawAxis(1);
    //       }
    //     }, 
    //     () -> {
    //       if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
    //         return -controller.getRawAxis(0);
    //       } else {
    //         return controller.getRawAxis(0);
    //       }
    //     }, 
    //     () -> MathUtil.clamp(m_swerve.getHeadingFeedback(new Rotation2d(m_swerve.getAutoAimingAngle())), -1, 1), true)
    // ));

    // AB.onFalse(new SetSpeed(10));    

    // YB.whileTrue(m_elevator.extensionRoutine.quasistatic(Direction.kForward));
    // AB.whileTrue(m_elevator.extensionRoutine.quasistatic(Direction.kReverse));
    
    // BB.whileTrue(m_elevator.extensionRoutine.dynamic(Direction.kForward));
    // XB.whileTrue(m_elevator.extensionRoutine.dynamic(Direction.kReverse));

    // upButton.onTrue(PathFindingWithPath.pathFindingAutoBuilder("Stage Middle Finisher", upButton));
    // downButton.onTrue(PathFindingWithPath.pathFindingAutoBuilder("AMP Finisher", downButton));

    windowButton.onTrue(new ResetOdo());
    
    YB.onTrue(new ReachState("anti-defense", false, 0));

    B3.onTrue(new ReachState("amp", false, 0));
    B1.whileTrue(new InstantCommand(() -> {
      m_leds.setState("amplify");
    }));
    B1.onFalse(new InstantCommand(() -> {
      m_leds.setState("idle");
    }));

    B4.onTrue(new ReachState("intake", false, 0));

    XB.onTrue(new ReachState("in", false, 0));
    rightBumper.whileTrue(new SequentialCommandGroup(
      new ReachState("intake", false, 0), 
      new SetIntakeSpeed(5),
      new ReachState("in", false, 0)
    ));

    rightBumper.onFalse(new SequentialCommandGroup(
      new ReachState("in", false, 0),      
      new InstantCommand(() -> {
        m_leds.setState("idle");
      }),
      new InstantCommand(() -> {RobotContainer.m_shooter.setIntakeVoltage(0);})
    ));


    leftBumper.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_leds.setState("rainbow");
      }),
      new SetShooterSpeed(9, true, 72)
    ));

    
    leftBumper.onFalse(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_leds.setState("idle");
      }),
      new SetShooterSpeed(0, false, 0)
    ));

    B1.onTrue(new SetShooterSpeed(4, true, 35));
    B1.onFalse(new SetShooterSpeed(0, false, 0));

    B2.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_leds.setState("rainbow");
      }),
      new SetShooterSpeed(6, true, 47)
    ));
    
    B2.onFalse(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_leds.setState("idle");
      }),
      new SetShooterSpeed(0, false, 0)
    ));

    BB.onTrue(new InstantCommand(() -> {RobotContainer.m_shooter.setIntakeVoltage(-4);}));
    BB.onFalse(new InstantCommand(() -> {RobotContainer.m_shooter.setIntakeVoltage(0);}));

    // B5.onTrue(new SequentialCommandGroup(
    //   new RightRatchet(true),
    //   new WaitCommand(0.5),
    //   new RightMotor(0.4)
    // )); 

    // B7.onTrue(new SequentialCommandGroup(
    //   new RightRatchet(false),
    //   new RightMotor(-0.4)
    // ));




    // B4.onTrue(new SequentialCommandGroup(
    //   new LeftRatchet(true),
    //   new WaitCommand(0.5),
    //   new LeftMotor(0.4)
    // )); 

    // B6.onTrue(new SequentialCommandGroup(
    //   new LeftRatchet(false),
    //   new LeftMotor(-0.4)
    // ));


    B5.onTrue(new SequentialCommandGroup(new ActivateRatchet(false), new WaitCommand(0.2), new RaiseClimber(0.4)));
    B5.onFalse(new SequentialCommandGroup(new RaiseClimber(0), new ActivateRatchet(true)));

    B7.onTrue(new SequentialCommandGroup(new ActivateRatchet(true), new RaiseClimber(-0.6)));
    B7.onFalse(new SequentialCommandGroup(new RaiseClimber(0), new ActivateRatchet(true)));
  }

  public Command getAutonomousCommand() {
    return m_chooser.get();
  }
}