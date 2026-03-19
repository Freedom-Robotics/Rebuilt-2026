package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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

/** AdvantageKit Shooter Subsystem, capable of replaying the shooter. */
public class Shooter extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class ShooterInputs {

    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
    public AngularVelocity velocity2 = DegreesPerSecond.of(0);
    public AngularVelocity setpoint2 = DegreesPerSecond.of(0);
    public Voltage volts2 = Volts.of(0);
    public Current current2 = Amps.of(0);
  }

  private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();

  private final SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);

  private final SparkMax secMotor = new SparkMax(12, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(0, 0, 0)
          .withSimClosedLoopController(0.04, 0, 0)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(60 / 40)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0, 0.25, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.285, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SparkMaxConfig secConfig = new SparkMaxConfig();

  private final SmartMotorController motor =
      new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig(motor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  /** Update the AdvantageKit "inputs" (data coming from the SMC) */
  private void updateInputs() {
    shooterInputs.velocity = shooter.getSpeed();
    shooterInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    shooterInputs.volts = motor.getVoltage();
    shooterInputs.current = motor.getStatorCurrent();
    shooterInputs.velocity2 = RPM.of(secMotor.getEncoder().getVelocity());
    shooterInputs.setpoint2 = RPM.of(secMotor.getClosedLoopController().getSetpoint());
    shooterInputs.volts2 = Volts.of(secMotor.getBusVoltage());
    shooterInputs.current2 = Current.ofBaseUnits(secMotor.getOutputCurrent(), Amps);
  }

  public Shooter() {
    secConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).follow(11, true);

    secMotor.configure(secConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Gets the current velocity of the shooter.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {
    return shooterInputs.velocity;
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Shooter/Setpoint", speed);
    return shooter.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    Logger.recordOutput("Shooter/DutyCycle", dutyCycle);
    return shooter.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return shooter.setSpeed(
        () -> {
          Logger.recordOutput("Shooter/Setpoint", speed.get());
          return speed.get();
        });
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return shooter.set(
        () -> {
          Logger.recordOutput("Shooter/DutyCycle", dutyCycle.get());
          return dutyCycle.get();
        });
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Shooter", shooterInputs);
    shooter.updateTelemetry();
  }
}
