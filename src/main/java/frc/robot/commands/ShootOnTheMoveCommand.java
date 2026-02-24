package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Largely written by Eeshwar based off their blog at
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends Command {

  // Subsystems
  private final TurretSubsystem turretSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  /** Current robot pose. (Blue-alliance) */
  private final Supplier<Pose2d> robotPose;

  /** Current robot-oriented chassis speeds. */
  private final Supplier<ChassisSpeeds> robotOrientedChassisSpeeds;

  /** Target goal pose */
  private final Supplier<Pose2d> goalPose;

  // Tuned Constants
  // TODO (jballock): Tune this
  // Distance -> RPM, Hood Angle, Time Of Flight
  private static final InterpolatingMatrixTreeMap<Double, N3, N1> SHOOTER_MAP =
      new InterpolatingMatrixTreeMap<>();

  static {
    SHOOTER_MAP.put(1.5, VecBuilder.fill(2400.0, 70.0, 0.38));
    SHOOTER_MAP.put(2.0, VecBuilder.fill(2500.0, 65.0, 0.45));
    SHOOTER_MAP.put(2.5, VecBuilder.fill(2550.0, 63.0, 0.52));
    SHOOTER_MAP.put(3.0, VecBuilder.fill(2650.0, 59.0, 0.60));
    SHOOTER_MAP.put(3.5, VecBuilder.fill(2850.0, 57.0, 0.68));
    SHOOTER_MAP.put(4.0, VecBuilder.fill(3000.0, 54.0, 0.76));
    SHOOTER_MAP.put(4.5, VecBuilder.fill(3250.0, 51.0, 0.85));
    SHOOTER_MAP.put(5.0, VecBuilder.fill(3400.0, 49.0, 0.94));
    SHOOTER_MAP.put(5.5, VecBuilder.fill(3550.0, 47.0, 0.96));
    SHOOTER_MAP.put(6.0, VecBuilder.fill(3650.0, 46.0, 0.98));
    SHOOTER_MAP.put(7.0, VecBuilder.fill(3750.0, 45.0, 1.01));
  }

  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double latency = 0.15;

  /**
   * Shoot on the move command to always have the turret ready to fire.
   *
   * @param turret Turret subsystem
   * @param hood Hood subsystem
   * @param flyWheel Flywheel subsystem
   * @param currentPose Current robot pose.
   * @param robotOrientedChassisSpeeds Current robot-oriented chassis speeds.
   * @param goalPoseSupplier Goal to shoot at.
   */
  public ShootOnTheMoveCommand(
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flyWheel,
      Supplier<Pose2d> currentPose,
      Supplier<ChassisSpeeds> robotOrientedChassisSpeeds,
      Supplier<Pose2d> goalPoseSupplier) {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    robotPose = currentPose;
    this.robotOrientedChassisSpeeds = robotOrientedChassisSpeeds;
    goalPose = goalPoseSupplier;

    setName("Shoot at hub on the move");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    var robotSpeed = robotOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d robotVelocity =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    //         .times(latency);
    // Translation2d futurePos = robotPose.get().getTranslation().plus(robotVelocity);

    Translation2d robotVelFieldFrame = robotVelocity.rotateBy(robotPose.get().getRotation());
    Translation2d futurePos =
        robotPose.get().getTranslation().plus(robotVelFieldFrame.times(latency));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.get().getTranslation();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();
    Translation2d targetDirection = targetVec.div(dist);

    // 3. Get shooter params
    Matrix<N3, N1> baseline = SHOOTER_MAP.get(dist);
    double baselineVelocity = dist / baseline.get(2, 0);

    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelFieldFrame);

    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();
    double velocityRatio = requiredVelocity / baselineVelocity;

    // Split the correction: sqrt gives equal "contribution" from each
    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    // Apply RPM scaling
    double adjustedRpm = baseline.get(0, 0) * rpmFactor;

    // Apply hood adjustment (changes horizontal component)
    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.get(1, 0)));
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.acos(ratio);

    // 6.5. Correct field relative turret angle to robot relative
    Rotation2d robotRelativeAngle = turretAngle.minus(robotPose.get().getRotation());

    // 7. SET OUTPUTS
    SmartDashboard.putNumber("Target/Turret Angle", turretAngle.getDegrees());
    SmartDashboard.putNumber("Target/Hood Angle", Math.toDegrees(adjustedHood));
    SmartDashboard.putNumber("Target/RPM", adjustedRpm);

    turretSubsystem.setAngleDirect(Degrees.of(robotRelativeAngle.getDegrees()));
    hoodSubsystem.setAngleDirect(Radians.of(adjustedHood));
    flywheelSubsystem.setRPMDirect(RotationsPerSecond.of(adjustedRpm / 60));
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
