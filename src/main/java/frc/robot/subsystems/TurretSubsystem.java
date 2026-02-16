package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.ID);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(TurretConstants.PID_CONTROLLER)
          // Configure Motor and Mechanism properties
          .withGearing(TurretConstants.GEARING)
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.125))
          .withOpenLoopRampRate(Seconds.of(0.125));
  private final SmartMotorController turretSMC =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(TurretConstants.START_ANGLE) // Starting position of the Pivot
          .withHardLimit(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)
          .withSoftLimits(TurretConstants.SOFT_MIN_ANGLE, TurretConstants.SOFT_MAX_ANGLE)
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
          .withMOI(TurretConstants.M_OF_INERTIA); // MOI Calculation

  private final Pivot turret = new Pivot(turretConfig);

  public TurretSubsystem() {}

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    double clampedDegrees =
        MathUtil.clamp(
            angle.in(Degrees),
            TurretConstants.MIN_ANGLE.in(Degrees),
            TurretConstants.MAX_ANGLE.in(Degrees));

    turretSMC.setPosition(Degrees.of(clampedDegrees));
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command sysId() {
    return turret.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
