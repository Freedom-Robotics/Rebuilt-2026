package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/** AdvantageKit Indexer Subsystem, capable of replaying the indexer. */
public class Indexer extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class IndexerInputs {

    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final IndexerInputsAutoLogged indexerInputs = new IndexerInputsAutoLogged();

  private final SparkMax someMotor = new SparkMax(13, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(0, 0, 0)
          .withSimClosedLoopController(0.0001, 0, 0)
          .withFeedforward(new SimpleMotorFeedforward(0, 0.3786, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(36 / 12)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false);

  private final SmartMotorController motor =
      new SparkWrapper(someMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConf =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(3))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(1000))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("IndexerMech", TelemetryVerbosity.HIGH);

  private FlyWheel indexer = new FlyWheel(flywheelConf);

  /** Update the AdvantageKit "inputs" (data coming from the SMC) */
  private void updateInputs() {
    indexerInputs.velocity = motor.getMechanismVelocity();
    indexerInputs.volts = motor.getVoltage();
    indexerInputs.current = motor.getStatorCurrent();
  }

  public Indexer() {}

  public AngularVelocity getVelocity() {
    return indexer.getSpeed();
  }

  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {
    indexer.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return indexer.run(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return indexer.set(dutyCycle);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    indexer.simIterate();
  }

  /**
   * Set the voltage of the indexer.
   *
   * @param volts Voltage to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVoltage(Voltage volts) {
    return run(() -> {
          Logger.recordOutput("Indexer/Voltage", volts);
          motor.setVoltage(volts);
        })
        .withName("IndexerSetVoltage");
  }

  /**
   * DutyCycle supplier controlling the indexer
   *
   * @param dutyCycle Dutycyle supplier
   * @return Command
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return run(() -> {
          Logger.recordOutput("Indexer/DutyCycle", dutyCycle.get());
          motor.setDutyCycle(dutyCycle.get());
        })
        .withName("IndexerSetDutyCycleSupplier");
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Indexer", indexerInputs);
    motor.updateTelemetry();
    indexer.updateTelemetry();
  }
}
