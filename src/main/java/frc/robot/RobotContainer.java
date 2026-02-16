// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.generated.SwerveTunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private double MaxSpeed =
      1.0
          * SwerveTunerConstants.kSpeedAt12Volts.in(
              MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  public final SwerveSubystem drivebase = SwerveTunerConstants.createDrivetrain();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Photon Vision subsystem
  public VisionSubsystem vision = new VisionSubsystem();

  // Super class for hood, turret, and flywheel
  public ShooterSubsystem shooter = new ShooterSubsystem(vision);

  // Class for intake
  public IntakeSubsystem intake = new IntakeSubsystem();

  // Class for intake
  public SerializerSubystem serializer = new SerializerSubystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final SendableChooser<Pose2d> passTargetChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set the default auto
    autoChooser.setDefaultOption("Straight Line", drivebase.getAutonomousCommand("New Auto"));

    // Goes to shooting point and stops
    autoChooser.addOption("Simple Shooter", drivebase.getAutonomousCommand("Simple Shooter Auto"));

    autoChooser.addOption(
        "Shooter + Ingest", drivebase.getAutonomousCommand("Shooter + Ingest Auto"));
    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Standard targets
    passTargetChooser.setDefaultOption("Right Side", new Pose2d(3.0, 6.5, new Rotation2d()));
    passTargetChooser.addOption("Left Side", new Pose2d(3.0, 1.5, new Rotation2d()));
    SmartDashboard.putData("Shooting Target", passTargetChooser);

    if (autoChooser.getSelected() == null) {
      RobotModeTriggers.autonomous()
          .onTrue(Commands.runOnce(() -> drivebase.seedFieldCentric(Rotation2d.kZero)));
    }
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
    if (Robot.isSimulation()) {
      drivebase.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivebase.applyRequest(
              () ->
                  drive
                      .withVelocityX(-driverXbox.getLeftX() * MaxSpeed)
                      .withVelocityY(driverXbox.getLeftY() * MaxSpeed)
                      .withRotationalRate(
                          driverXbox.getRightX()
                              * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));

    } else {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivebase.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivebase.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          -driverXbox.getLeftY()
                              * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          -driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(
                          -driverXbox.getRightX()
                              * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));
    }

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivebase.applyRequest(() -> idle).ignoringDisable(true));

    drivebase.registerTelemetry(logger::telemeterize);

    Command hubShootOnMoveCommand =
        new ShootOnTheMoveCommand(
            shooter.turret,
            shooter.hood,
            shooter.flywheel,
            () -> drivebase.getState().Pose,
            () -> drivebase.getState().Speeds,
            () -> getHubTarget());

    Command customShootOnMoveCommand =
        new ShootOnTheMoveCommand(
            shooter.turret,
            shooter.hood,
            shooter.flywheel,
            () -> drivebase.getState().Pose,
            () -> drivebase.getState().Speeds,
            () -> AllianceFlipUtil.apply(passTargetChooser.getSelected()));

    if (Robot.isSimulation()) {
      driverXbox.y().whileTrue(hubShootOnMoveCommand);
      driverXbox.b().whileTrue(customShootOnMoveCommand);
      driverXbox
          .rightBumper()
          .onFalse(intake.retract())
          .whileTrue(intake.intake(Degrees.of(-20.0)));
      driverXbox
          .x()
          .whileTrue(serializer.serializeAndKick(1.0))
          .onFalse(serializer.stopSerialize());

      // Manual turret control
      driverXbox
          .rightTrigger()
          .onFalse(shooter.turret.setDutyCycle(0.0))
          .whileTrue(shooter.turret.setDutyCycle(0.125));
      driverXbox
          .leftTrigger()
          .onFalse(shooter.turret.setDutyCycle(0.0))
          .whileTrue(shooter.turret.setDutyCycle(-0.125));
    }
    if (DriverStation.isTest()) {
      // Buttons for intake testing
      driverXbox.a().onFalse(intake.rawIntakeControl(0.0)).whileTrue(intake.rawIntakeControl(0.25));
      driverXbox.b().onFalse(intake.rawArmControl(0.0)).whileTrue(intake.rawArmControl(0.125));

      // Button for serializer
      driverXbox.x().onFalse(serializer.stopSerialize()).whileTrue(serializer.serialize(0.25));

      // Buttons for turret
      driverXbox
          .rightBumper()
          .onFalse(shooter.turret.setDutyCycle(0.0))
          .whileTrue(shooter.turret.setDutyCycle(0.125));
      driverXbox
          .leftBumper()
          .onFalse(shooter.turret.setDutyCycle(0.0))
          .whileTrue(shooter.turret.setDutyCycle(-0.125));

      // Button for flywheel
      driverXbox
          .y()
          .onFalse(shooter.flywheel.setDutyCycle(0.0))
          .whileTrue(shooter.flywheel.setDutyCycle(0.25));

      // Button for hood
      driverXbox
          .rightTrigger()
          .onFalse(shooter.hood.setDutyCycle(0.0))
          .whileTrue(shooter.hood.setDutyCycle(0.125));
      driverXbox
          .leftTrigger()
          .onFalse(shooter.hood.setDutyCycle(0.0))
          .whileTrue(shooter.hood.setDutyCycle(-0.125));

    } else {
      // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    if (brake) {
      drivebase.applyRequest(() -> this.brake);
    }
  }

  public Pose2d getHubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.BLUE_HUB);
  }
}
