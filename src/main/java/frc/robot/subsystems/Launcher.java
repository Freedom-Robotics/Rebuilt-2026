package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

/** AdvantageKit Launcher Subsystem, capable of replaying the launcher. */
public class Launcher extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class LauncherInputs {

    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final LauncherInputsAutoLogged launcherInputs = new LauncherInputsAutoLogged();

  private final SparkMax armMotor = new SparkMax(12, MotorType.kBrushless);

  private final SparkMax secMotor = new SparkMax(11, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(1, 0, 0, RPM.of(10000), RPM.per(Second).of(60))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(60 / 40)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("LauncherMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withFeedforward(new SimpleMotorFeedforward(0.2, 0.12, 0.01))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private SparkMaxConfig secConfig = new SparkMaxConfig();

  private final SmartMotorController motor =
      new SparkWrapper(armMotor, DCMotor.getNEO(2), motorConfig);
  private final FlyWheelConfig launcherConfig =
      new FlyWheelConfig(motor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(1))
          .withTelemetry("LauncherMech", TelemetryVerbosity.HIGH);
  private final FlyWheel launcher = new FlyWheel(launcherConfig);

  /** Update the AdvantageKit "inputs" (data coming from the SMC) */
  private void updateInputs() {
    launcherInputs.velocity = launcher.getSpeed();
    launcherInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    launcherInputs.volts = motor.getVoltage();
    launcherInputs.current = motor.getStatorCurrent();
  }

  public Launcher() {
    secConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).follow(12, true);

    secMotor.configure(secConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Gets the current velocity of the launcher.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {
    return launcherInputs.velocity;
  }

  /**
   * Set the launcher velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Launcher/Setpoint", speed);
    return launcher.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the launcher.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    Logger.recordOutput("Launcher/DutyCycle", dutyCycle);
    return launcher.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return launcher.setSpeed(
        () -> {
          Logger.recordOutput("Launcher/Setpoint", speed.get());
          return speed.get();
        });
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return launcher.set(
        () -> {
          Logger.recordOutput("Launcher/DutyCycle", dutyCycle.get());
          return dutyCycle.get();
        });
  }

  @Override
  public void simulationPeriodic() {
    launcher.simIterate();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Launcher", launcherInputs);
    launcher.updateTelemetry();
  }
}
