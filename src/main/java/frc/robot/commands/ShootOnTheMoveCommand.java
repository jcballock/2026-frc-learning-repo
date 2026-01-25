package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
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

  /** Current field-oriented chassis speeds. */
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;

  /** Pose to shoot at. */
  private final Pose2d goalPose;

  // Tuned Constants
  // TODO (jballock): Tune this
  // Distance -> RPM, Hood Angle, Time Of Flight
  private static final InterpolatingMatrixTreeMap<Double, N3, N1> SHOOTER_MAP =
      new InterpolatingMatrixTreeMap<>();

  static {
    SHOOTER_MAP.put(1.5, VecBuilder.fill(2800.0, 35.0, 0.38));
    SHOOTER_MAP.put(2.0, VecBuilder.fill(3100.0, 38.0, 0.45));
    SHOOTER_MAP.put(2.5, VecBuilder.fill(3400.0, 42.0, 0.52));
    SHOOTER_MAP.put(3.0, VecBuilder.fill(3650.0, 46.0, 0.60));
    SHOOTER_MAP.put(3.5, VecBuilder.fill(3900.0, 50.0, 0.68));
    SHOOTER_MAP.put(4.0, VecBuilder.fill(4100.0, 54.0, 0.76));
    SHOOTER_MAP.put(4.5, VecBuilder.fill(4350.0, 58.0, 0.85));
    SHOOTER_MAP.put(5.0, VecBuilder.fill(4550.0, 62.0, 0.94));
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
   * @param fieldOrientedChassisSpeeds Current field-oriented chassis speeds.
   * @param goal Goal to shoot at.
   */
  public ShootOnTheMoveCommand(
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flyWheel,
      Supplier<Pose2d> currentPose,
      Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
      Pose2d goal) {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    setName("Shoot on the move");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d robotVelocity =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .times(latency);
    Translation2d futurePos = robotPose.get().getTranslation().plus(robotVelocity);

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();
    Translation2d targetDirection = targetVec.div(dist);

    // 3. Get shooter params
    Matrix<N3, N1> baseline = SHOOTER_MAP.get(dist);
    double baselineVelocity = dist / baseline.get(2, 0);

    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

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

    // 7. SET OUTPUTS
    System.out.println(
        "Turret Angle: "
            + Degrees.of(turretAngle.getDegrees())
            + " Speed: "
            + MetersPerSecond.of(totalVelocity)
            + " Hood Angle: "
            + Radians.of(adjustedHood)
            + " Dist (m): "
            + dist
            + " Ideal Speed: "
            + targetHorizFromHood
            + " Robot Speed: "
            + robotSpeed);

    // TODO (jballock): Respect hard limits / use proper commands
    turretSubsystem.setAngleDirect(Degrees.of(turretAngle.getDegrees()));
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
