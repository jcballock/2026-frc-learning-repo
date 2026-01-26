package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;
import java.util.function.Supplier;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeArmMotor = new TalonFX(IntakeArmConstants.ID);
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.ID);
  private static final double intakeMotorSimGearRatio = 10.0;
  private final DCMotorSim intakeMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, intakeMotorSimGearRatio),
          DCMotor.getKrakenX60Foc(1));

  private final SmartMotorControllerConfig intakeArmMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(IntakeArmConstants.PID_CONTROLLER)
          .withGearing(IntakeArmConstants.GEARING)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController intakeArmSMC =
      new TalonFXWrapper(intakeArmMotor, DCMotor.getKrakenX60(1), intakeArmMotorConfig);

  private final ArmConfig intakeArmConfig =
      new ArmConfig(intakeArmSMC)
          .withMass(IntakeArmConstants.MASS)
          .withStartingPosition(IntakeArmConstants.START_ANGLE)
          .withLength(IntakeArmConstants.LENGTH)
          .withTelemetry("IntakeArmMech", TelemetryVerbosity.HIGH)
          .withMOI(IntakeArmConstants.M_OF_INERTIA) // Required for SIM
          .withHardLimit(IntakeArmConstants.MIN_ANGLE, IntakeArmConstants.MAX_ANGLE);

  private final Arm intakeArm = new Arm(intakeArmConfig);

  public IntakeSubsystem() {}

  public Command setAngle(Angle angle) {
    return intakeArm.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    intakeArmSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return intakeArm.setAngle(angleSupplier);
  }

  public Command intake(Angle angle) {
    return runOnce(() -> intakeMotor.set(1.0)).andThen(intakeArm.setAngle(angle));
  }

  public Command intake(Supplier<Angle> angleSupplier) {
    return runOnce(() -> intakeMotor.set(1.0)).andThen(intakeArm.setAngle(angleSupplier));
  }

  public Command outtake(Supplier<Angle> angleSupplier) {
    return runOnce(() -> intakeMotor.set(-1.0)).andThen(intakeArm.setAngle(angleSupplier));
  }

  public Command retract() {
    return runOnce(() -> intakeMotor.setControl(new NeutralOut()))
        .andThen(intakeArm.setAngle(Degrees.of(90)));
  }

  public Angle getAngle() {
    return intakeArm.getAngle();
  }

  public Command sysId() {
    return intakeArm.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return intakeArm.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return intakeArm.set(dutyCycle);
  }

  @Override
  public void periodic() {
    intakeArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakeArm.simIterate();
    var talonFXSim = intakeMotor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    intakeMotorSim.setInputVoltage(motorVoltage.in(Volts));
    intakeMotorSim.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(
        intakeMotorSim.getAngularPosition().times(intakeMotorSimGearRatio));
    talonFXSim.setRotorVelocity(intakeMotorSim.getAngularVelocity().times(intakeMotorSimGearRatio));
  }
}
