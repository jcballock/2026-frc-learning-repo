package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootAndMoveCommandGroup extends SequentialCommandGroup {
  public ShootAndMoveCommandGroup(
      ShooterSubsystem shooter, SwerveSubsystem drivebase, CommandXboxController xbox) {
    addCommands(
        new ParallelCommandGroup(
            // runs l4 sequence
            new ShootOnTheMoveCommand(
                shooter.turret,
                shooter.hood,
                shooter.flywheel,
                drivebase::getPose,
                drivebase::getFieldVelocity,
                new Pose2d(4.6, 4, Rotation2d.fromDegrees(0))),
            new AbsoluteFieldDrive(
                drivebase, xbox::getLeftX, xbox::getLeftY, drivebase.getHeading()::getRadians)));
  }
}
