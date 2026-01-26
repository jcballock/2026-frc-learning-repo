package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import java.util.function.Supplier;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.ID);

  private final SmartMotorControllerConfig hoodMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(HoodConstants.PID_CONTROLLER)
          .withGearing(HoodConstants.GEARING)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController hoodSMC =
      new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX60(1), hoodMotorConfig);

  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withMass(HoodConstants.MASS)
          .withStartingPosition(HoodConstants.START_ANGLE)
          .withLength(HoodConstants.LENGTH)
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
          .withMOI(HoodConstants.M_OF_INERTIA) // Required for SIM
          .withHardLimit(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

  // The Hood can be modeled as an arm since it has
  // gravitational force acted upon based on the angle its i
  private final Arm hood = new Arm(hoodConfig);

  public HoodSubsystem() {}

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    hoodSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return hood.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command sysId() {
    return hood.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return hood.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
