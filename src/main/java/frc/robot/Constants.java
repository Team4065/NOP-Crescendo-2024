// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final int pdhCANID = 18;

  public static class FieldConstants {
    public static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Translation2d blueSpeakerReferencePoint = new Translation2d(Units.inchesToMeters(36.179), 5.475);
    public static final Translation2d redSpeakerReferencePoint = new Translation2d(Units.inchesToMeters(651.223), 5.475);
  }

  public static final int ledPWMPort = 9;
  public static final int ledLength = 75;

  public static final Field2d displayField = new Field2d();
  public static final HashMap<Command, String> autoRoutines = new HashMap<>(); // HashMap to find the auto name based on what command is selected in SendableChooser
  public static final ShuffleboardTab dataTab = Shuffleboard.getTab("data");

  public static class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final int encoderTicks = 2048; // Encoder ticks for CanCoder and REV Encoder 

    public static final double wheel_radius_meters = Units.inchesToMeters(2);

    public static final double MAX_SPEED_FEET = 14;

    public static class ModuleConstants {
      // FRONT LEFT
      public static final int frontLeftDriveID = 4;
      public static final int frontLeftTurnID = 5;
      public static final int frontLeftCanCoderID = 6;
      public static final Rotation2d frontLeftEncoderOffset = new Rotation2d(-2.141437179888355);

      // FRONT RIGHT
      public static final int frontRightDriveID = 1;
      public static final int frontRightTurnID = 2;
      public static final int frontRightCanCoderID = 3;
      public static final Rotation2d frontRightEncoderOffset =  new Rotation2d(-1.5033011721279284);

      // BACK LEFT
      public static final int backLeftDriveID = 7;
      public static final int backLeftTurnID = 8;
      public static final int backLeftCanCoderID = 9;
      public static final Rotation2d backLeftEncoderOffset = new Rotation2d(-0.3512816004258118);

      // BACK RIGHT
      public static final int backRightDriveID = 10;
      public static final int backRightTurnID = 11;
      public static final int backRightCanCoderID = 12;
      public static final Rotation2d backRightEncoderOffset =  new Rotation2d(-1.9220779272207087);
    }
  }

  public static class ElevatorConstants {
    public static final DCMotor extensionMotorSimGearbox = DCMotor.getFalcon500(2);
    public static final double extensionGearRatio = 10.0;
    public static final double extensionCarriageMassKg = 4;
    public static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double fullyInExtension = Units.inchesToMeters(27);
    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = Units.feetToMeters(1.75);
    public static final int extenstionMotorID = 13;
    public static final int encoderAChannel = 0;
    public static final int encoderBChannel = 1;
    public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 1024;

    public static final DCMotor tiltMotorSimGearbox = DCMotor.getFalcon500(1);
    public static final double tiltGearRatio = 133;
    public static final double armMass = 8.61826;
    public static final double minAngleRadians = Units.degreesToRadians(-10);
    public static final double maxAngleRadians = Units.degreesToRadians(90);
    public static int encoderCChannel = 3;
    public static int encoderDChannel = 4;
    public static final double tiltEncoderDistPerPulse = 2.0 * Math.PI / 1024;

    public static final int rightTiltMotorCANID = 13;
    public static final int leftTiltMotorCANID = 14;
    public static final int extensionMotorCANID = 21;

    public static final int absoluteEncoderDIO = 9;
    public static final int neutralModeButton = 6;
  }

  public static class ShooterConstants {
    public static final int topRollerCANID = 22;
    public static final int bottomRollerCANID = 16;

    public static final int intakeMotorCANID = 18;
    public static final int beamBreakDIO = 0;
  }

  public static class ClimberConstants {
    public static final int rightTelescopeCANID = 20;
    public static final int leftTelescopeCANID = 15;

    public static final int pcmCANID = 19;
    public static final int rightRatchetPort = 0;
    public static final int leftRatchetPort = 1;
  }

  public static class LimelightPositions {
    public static final Transform3d camPosBR = new Transform3d(new Translation3d(-Units.inchesToMeters(4.5), -Units.inchesToMeters(12), Units.inchesToMeters(15)), new Rotation3d(Units.degreesToRadians(0), -Units.degreesToRadians(25), Units.degreesToRadians(180 - 35)));
    public static final Transform3d camPosBL = new Transform3d(new Translation3d(-Units.inchesToMeters(4.5), Units.inchesToMeters(12), Units.inchesToMeters(15)), new Rotation3d(-Units.degreesToRadians(0), -Units.degreesToRadians(25), Units.degreesToRadians(180 + 35)));
    public static final Transform3d camPosNoteCamR = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(10),
        -Units.inchesToMeters(11.5),
        Units.inchesToMeters(8)
      ),
      new Rotation3d(0, -Units.degreesToRadians(10), Units.degreesToRadians(15))
    );
    public static final Transform3d camPosNoteCamL = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(10),
        Units.inchesToMeters(11.5),
        Units.inchesToMeters(8)
      ),
      new Rotation3d(0, -Units.degreesToRadians(10), -Units.degreesToRadians(15))
    );
  }
}
