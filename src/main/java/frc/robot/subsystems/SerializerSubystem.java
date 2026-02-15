package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;

public class SerializerSubystem extends SubsystemBase {
  private final TalonFX kickerMotor = new TalonFX(IntakeArmConstants.ID);
  private final TalonFX serializerMotor = new TalonFX(IntakeConstants.ID);

  private static final double kickerMotorSimGearRatio = 10.0;
  private final DCMotorSim kickerMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.001, kickerMotorSimGearRatio),
          DCMotor.getKrakenX44Foc(1));

  private static final double serializerMotorSimGearRatio = 10.0;
  private final DCMotorSim serializerMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.001, serializerMotorSimGearRatio),
          DCMotor.getKrakenX60Foc(1));

  public SerializerSubystem() {}

  // Speed is between -1.0 and 1.0
  public Command serialize(double speed) {
    return runOnce(() -> serializerMotor.set(speed));
  }

  // Speed is between -1.0 and 1.0
  public Command serializeAndKick(double speed) {
    return runOnce(() -> serializerMotor.set(speed)).andThen(() -> kickerMotor.set(speed));
  }

  public Command stopSerialize() {
    return runOnce(() -> serializerMotor.setControl(new NeutralOut()))
        .andThen(() -> kickerMotor.setControl(new NeutralOut()));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    var serializerSim = serializerMotor.getSimState();
    serializerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    serializerMotorSim.setInputVoltage(serializerSim.getMotorVoltageMeasure().in(Volts));
    serializerMotorSim.update(0.020); // assume 20 ms loop time
    serializerSim.setRawRotorPosition(
        serializerMotorSim.getAngularPosition().times(serializerMotorSimGearRatio));
    serializerSim.setRotorVelocity(
        serializerMotorSim.getAngularVelocity().times(serializerMotorSimGearRatio));

    var kickerSim = kickerMotor.getSimState();
    kickerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    kickerMotorSim.setInputVoltage(kickerSim.getMotorVoltageMeasure().in(Volts));
    kickerMotorSim.update(0.020); // assume 20 ms loop time
    kickerSim.setRawRotorPosition(
        kickerMotorSim.getAngularPosition().times(kickerMotorSimGearRatio));
    kickerSim.setRotorVelocity(kickerMotorSim.getAngularVelocity().times(kickerMotorSimGearRatio));
  }
}
