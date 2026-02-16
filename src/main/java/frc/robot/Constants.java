// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import swervelib.math.Matter;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 110 * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  public static final double LOOP_TIME = 0.13;

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class FieldConstants {

    // Hold time on motor brakes when disabled
    public static final Pose2d BLUE_HUB = new Pose2d(4.6, 4, Rotation2d.fromDegrees(0));
    public static final double FIELD_LENGTH_M = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH_M = Units.inchesToMeters(317.69);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class HoodConstants {
    // ID
    public static final int ID = 35;

    // Hood Properties
    public static final Mass MASS = Mass.ofBaseUnits(2, Pounds);
    public static final Distance LENGTH = Distance.ofBaseUnits(7, Inches);
    public static final MomentOfInertia M_OF_INERTIA =
        MomentOfInertia.ofBaseUnits(0.00488, KilogramSquareMeters);
    public static final Angle START_ANGLE = Degrees.of(0);
    public static final Angle MIN_ANGLE = Degrees.of(0);
    public static final Angle MAX_ANGLE = Degrees.of(120);
    public static final MechanismGearing GEARING =
        new MechanismGearing(GearBox.fromReductionStages(3, 4));

    // Hood PID
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(
            20.0,
            0,
            0,
            new Constraints(
                DegreesPerSecond.of(180).in(RotationsPerSecond),
                DegreesPerSecondPerSecond.of(90).in(RotationsPerSecondPerSecond)));
  }

  public static class TurretConstants {
    // ID
    public static final int ID = 36;

    // CANcoder
    public static final int CANCODER_ID = 40;
    public static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    // Turret Properties
    public static final Mass MASS = Mass.ofBaseUnits(2, Pounds);
    public static final Distance LENGTH = Distance.ofBaseUnits(7, Inches);
    public static final MomentOfInertia M_OF_INERTIA =
        MomentOfInertia.ofBaseUnits(0.00956, KilogramSquareMeters);
    public static final Angle START_ANGLE = Degrees.of(0);
    public static final Angle MIN_ANGLE = Degrees.of(-95);
    public static final Angle MAX_ANGLE = Degrees.of(95);
    public static final Angle SOFT_MIN_ANGLE = Degrees.of(-90);
    public static final Angle SOFT_MAX_ANGLE = Degrees.of(90);
    public static final MechanismGearing GEARING =
        new MechanismGearing(GearBox.fromReductionStages(40));

    // CRT
    public static final double MATCH_TOLERANCE = 0.01;

    // Turret PID
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(
            150,
            0,
            0.25,
            new Constraints(
                DegreesPerSecond.of(360).in(RotationsPerSecond),
                DegreesPerSecondPerSecond.of(360).in(RotationsPerSecondPerSecond)));
  }

  public static class FlywheelConstants {
    // ID
    public static final int ID = 37;

    // TODO
  }

  public static class IntakeArmConstants {
    // ID
    public static final int ID = 38;

    // Intake Arm Properties
    public static final Mass MASS = Mass.ofBaseUnits(7, Pounds);
    public static final Distance LENGTH = Distance.ofBaseUnits(14.5, Inches);
    public static final MomentOfInertia M_OF_INERTIA =
        MomentOfInertia.ofBaseUnits(0.01438, KilogramSquareMeters);
    public static final Angle START_ANGLE = Degrees.of(90);
    public static final Angle MIN_ANGLE = Degrees.of(-20);
    public static final Angle MAX_ANGLE = Degrees.of(100);
    public static final MechanismGearing GEARING =
        new MechanismGearing(GearBox.fromReductionStages(4, 4));

    // Intake Arm PID
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(
            10.0,
            0,
            0,
            new Constraints(
                DegreesPerSecond.of(360).in(RotationsPerSecond),
                DegreesPerSecondPerSecond.of(280).in(RotationsPerSecondPerSecond)));
  }

  public static class IntakeConstants {
    // ID
    public static final int ID = 39;
  }
}
