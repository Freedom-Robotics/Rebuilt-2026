package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
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

/** AdvantageKit Intake Subsystem, capable of replaying the intake. */
public class Intake extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class IntakeInputs {

    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

  private final SparkMax armMotor = new SparkMax(20, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(1, 0, 0, RPM.of(10000), RPM.per(Second).of(60))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController motor =
      new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
  private final FlyWheelConfig intakeConfig =
      new FlyWheelConfig(motor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);
  private final FlyWheel intake = new FlyWheel(intakeConfig);

  /** Update the AdvantageKit "inputs" (data coming from the SMC) */
  private void updateInputs() {
    intakeInputs.velocity = intake.getSpeed();
    intakeInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    intakeInputs.volts = motor.getVoltage();
    intakeInputs.current = motor.getStatorCurrent();
  }

  public Intake() {}

  /**
   * Gets the current velocity of the intake.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {
    return intakeInputs.velocity;
  }

  /**
   * Set the intake velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Intake/Setpoint", speed);
    return intake.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the intake.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    Logger.recordOutput("Intake/DutyCycle", dutyCycle);
    return intake.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return intake.setSpeed(
        () -> {
          Logger.recordOutput("Intake/Setpoint", speed.get());
          return speed.get();
        });
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return intake.set(
        () -> {
          Logger.recordOutput("Intake/DutyCycle", dutyCycle.get());
          return dutyCycle.get();
        });
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Intake", intakeInputs);
    intake.updateTelemetry();
  }
}
