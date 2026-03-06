

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ClimbFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ClimbConfig;
import yams.mechanisms.positional.Climb;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbSubsystem extends SubsystemBase {

  public class ClimbConstants {

    public static final Angle SOME_ANGLE = Degrees.of(20);
    public static final Angle DOWN_ANGLE = Degrees.of(-35);
    public static final Angle L1_ANGLE = Degrees.of(65);
    public static final Angle HANDOFF_ANGLE = Degrees.of(135);
    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double VELOCITY = 458;
    public static final double ACCELERATION = 688;
    public static final int MOTOR_ID = 40;
    public static final double STATOR_CURRENT_LIMIT = 120;
    public static final double MOI = 0.1055457256;

  }

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to
   * the SMC is an Output. Everything coming
   * from the SMC is an Input.
   */
  @AutoLog
  public static class ClimbInputs {

    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Angle pivotDesiredPosition = Degrees.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);

  }

  private final ClimbInputsAutoLogged climbInputs = new ClimbInputsAutoLogged();

  private final TalonFX climbMotor = new TalonFX(ClimbConstants.MOTOR_ID);

  ///
  /// YAMS Configurations
  ///
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(ClimbConstants.KP,
          ClimbConstants.KI,
          ClimbConstants.KD,
          DegreesPerSecond.of(ClimbConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(ClimbConstants.ACCELERATION))
      .withSimClosedLoopController(ClimbConstants.KP,
          ClimbConstants.KI,
          ClimbConstants.KD,
          DegreesPerSecond.of(ClimbConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(ClimbConstants.ACCELERATION))
      .withFeedforward(new ClimbFeedforward(ClimbConstants.KS,
          ClimbConstants.KG,
          ClimbConstants.KV,
          ClimbConstants.KA))
      .withSimFeedforward(new ClimbFeedforward(ClimbConstants.KS,
          ClimbConstants.KG,
          ClimbConstants.KV,
          ClimbConstants.KA))
      .withTelemetry("", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)

      .withStatorCurrentLimit(Amps.of(ClimbConstants.STATOR_CURRENT_LIMIT));

  private SmartMotorController climbSMC = new TalonFXWrapper(climbMotor, DCMotor.getFalcon500(1), smcConfig);

  private ClimbConfig climbCfg = new ClimbConfig(climbSMC)
      .withHardLimit(Degrees.of(-25), Degrees.of(141))
      .withStartingPosition(Degrees.of(141))
      .withLength(Feet.of((14.0 / 12)))
      .withMOI(ClimbConstants.MOI)
      .withTelemetry("Climb", TelemetryVerbosity.HIGH);

  // Climb Mechanism
  private Climb climb = new Climb(climbCfg);

  /**
   * Updates AdvantageKit inputs from the {@link Climb} to be used in the rest of
   * the program.
   */
  public void updateInputs() {
    climbInputs.pivotPosition = climb.getAngle();
    climbInputs.pivotVelocity = climbSMC.getMechanismVelocity();
    climbInputs.pivotAppliedVolts = climbSMC.getVoltage();
    climbInputs.pivotCurrent = climbSMC.getStatorCurrent();
  }

  /**
   * Set the angle of the climb.
   *
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return climb.setAngle(angle);
  }

  /**
   * Move the climb up and down.
   *
   * @param dutycycle [-1, 1] speed to set the climb too.
   */
  // public Command set(double dutycycle) { return climb.set(dutycycle);}

  /**
   * Run sysId on the {@link Climb}
   */
  public Command sysId() {
    return climb.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    Logger.processInputs("Climb", climbInputs);
    climb.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    climb.simIterate();
  }

  @AutoLogOutput
  public Angle getAngleSetpoint() {
    return climbSMC.getMechanismPositionSetpoint().orElse(null);
  }

  public Angle getAngle() {
    return climbInputs.pivotPosition;
  }

  public AngularVelocity getVelocity() {
    return climbInputs.pivotVelocity;
  }

  public Angle getSetpointAngle() {
    return climbInputs.pivotDesiredPosition;
  }

  public Voltage getVoltage() {
    return climbInputs.pivotAppliedVolts;
  }

  public Current getCurrent() {
    return climbInputs.pivotCurrent;
  }
}
