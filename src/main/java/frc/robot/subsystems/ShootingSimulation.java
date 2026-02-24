package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FieldObject3D;
import frc.robot.util.ProjectileSimulator;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;
import java.util.function.Supplier;

public class ShootingSimulation extends SubsystemBase {
  Pose2d HUB_POSE = new Pose2d(4.0, 4.1, new Rotation2d());
  Random rand = new Random();
  private List<FieldObject3D> allFuel = new ArrayList<>();
  private double timeSinceLastShot = 0.0;
  Function<Double, Double> pitch;

  ProjectileSimulator sim = new ProjectileSimulator();

  Supplier<Pose2d> drivePoseSupplier;
  Supplier<Pose2d> driveVelocitySupplier;
  Supplier<Angle> turretAngleSupplier;
  Supplier<Angle> hoodAngleSupplier;
  Supplier<LinearVelocity> flywheelVelocitySupplier;

  public ShootingSimulation(
      Supplier<Pose2d> drivePose,
      Supplier<Pose2d> velocity,
      Supplier<Angle> turretAngle,
      Supplier<Angle> hoodAngle,
      Supplier<LinearVelocity> flywheelVelocity) {
    drivePoseSupplier = drivePose;
    driveVelocitySupplier = velocity;
    turretAngleSupplier = turretAngle;
    hoodAngleSupplier = hoodAngle;
    flywheelVelocitySupplier = flywheelVelocity;

    for (int i = 0; i < 12; i++) {
      allFuel.add(
          new FieldObject3D(
              String.format("Field/Fuel%d", i), String.format("Field/Fuel%dTime", i)));
    }
  }

  public void periodic() {
    for (int i = 0; i < allFuel.size(); i++) {
      FieldObject3D fuel = allFuel.get(i);
      if (fuel.getCount() == 0) {
        if (fuel.hasTrajectory() && timeSinceLastShot > 2) {
          fuel.setPose();
          timeSinceLastShot = 0;
        } else if (fuel.hasTrajectory() && timeSinceLastShot <= 2) {

        } else {
          fuel.setTrajectory(
              sim.simulate(
                  drivePoseSupplier.get().getTranslation(),
                  flywheelVelocitySupplier.get().in(MetersPerSecond) * 0.5, // 50% speed transfer
                  (turretAngleSupplier.get().in(Radians)
                      + drivePoseSupplier.get().getRotation().getRadians()),
                  hoodAngleSupplier.get().in(Radians),
                  driveVelocitySupplier
                      .get()
                      .getTranslation()
                      .plus(
                          Constants.robotToTurret
                              .rotateBy(
                                  drivePoseSupplier.get().getRotation().plus(Rotation2d.kCCW_90deg))
                              .times(driveVelocitySupplier.get().getRotation().getRadians()))));
        }
      } else {
        if (!fuel.setPose()) {
          fuel.setTrajectory(
              sim.simulate(
                  drivePoseSupplier
                      .get()
                      .getTranslation()
                      .plus(
                          Constants.robotToTurret.rotateBy(drivePoseSupplier.get().getRotation())),
                  flywheelVelocitySupplier.get().in(MetersPerSecond) * 0.5, // 50% speed transfer
                  (turretAngleSupplier.get().in(Radians)
                      + drivePoseSupplier.get().getRotation().getRadians()),
                  hoodAngleSupplier.get().in(Radians),
                  driveVelocitySupplier
                      .get()
                      .getTranslation()
                      .plus(
                          (Constants.robotToTurret.rotateBy(
                                  drivePoseSupplier
                                      .get()
                                      .getRotation()
                                      .plus(Rotation2d.kCCW_90deg)))
                              .times(driveVelocitySupplier.get().getRotation().getRadians()))));
        }
      }
    }
    timeSinceLastShot++;
  }
}
