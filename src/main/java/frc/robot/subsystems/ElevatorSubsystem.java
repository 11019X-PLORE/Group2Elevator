// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Dual-motor elevator on the ArmDemo control pattern, reduced to one fill-in limit. Both encoder
 * positions are direction-corrected (extension positive) and checked against the single
 * {@link Constants.Elevator#MAX_EXTENSION_ROTATIONS} travel — the only constant that must be
 * measured before the elevator moves. All limit logic runs in fractions of that travel, so the
 * sign of the measured value carries the winding direction and every voltage sign follows it.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private static final double MINIMUM_VALID_CALIBRATION_ROTATIONS = 1e-6;
  private static final double MINIMUM_VELOCITY_TIMESTEP_SECONDS = 1e-6;
  private static final double VELOCITY_FILTER_ALPHA = 0.3;

  private final ElevatorIO io;
  // Sensor inputs: filled by io.updateInputs() once per cycle, logged wholesale for replay.
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  // The one fill-in limit: direction-corrected motor rotations from the calibrated zero to full
  // extension (averaged over both motors at measurement time). Its sign is the measured
  // extension direction. Empty (0.0 or a mistyped constant) means uncalibrated: every movement
  // is refused until a real limit is filled in.
  private final OptionalDouble maxExtensionRotations;
  // Tuning inputs: polled once per cycle (see pollTunables) into the sanitized snapshots below.
  // The supplier fields are a test seam — production uses the LoggedNetworkNumber constructors.
  private final DoubleSupplier maxTravelFractionPerSecondSupplier;
  private final DoubleSupplier voltageSlewVoltsPerSecondSupplier;
  private double maxTravelFractionPerSecond;
  private double voltageSlewVoltsPerSecond;
  private double previousMotor1ExtensionFraction;
  private double previousMotor2ExtensionFraction;
  private double previousTimestampSeconds;
  private double motor1ExtensionFractionVelocityPerSecond;
  private double motor2ExtensionFractionVelocityPerSecond;
  // Signed, from motor 1's viewpoint (motor 2 always mirrors it): the ramp reference passes
  // through zero on a direction reversal instead of snapping to the opposite voltage.
  private double lastMovementVoltage;
  private double lastMovementTimestampSeconds;
  private double moveStartTimestampSeconds;
  private double lastMoveDurationSeconds = -1.0;
  private String lastMoveActionName = "";
  private boolean midMovementIsExtension;

  /** Production constructor: the subsystem owns its NetworkTables tuning entries. */
  public ElevatorSubsystem(ElevatorIO io, double maxExtensionRotations) {
    this(
        io,
        maxExtensionRotations,
        tunable(
            "/SmartDashboard/Elevator Max Travel (/s)",
            Constants.Elevator.DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND),
        tunable(
            "/SmartDashboard/Elevator Voltage Slew (V/s)",
            Constants.Elevator.DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND));
  }

  private static DoubleSupplier tunable(String key, double defaultValue) {
    LoggedNetworkNumber entry = new LoggedNetworkNumber(key, defaultValue);
    return entry::get;
  }

  // Test seam (package-private): constant suppliers instead of NetworkTables entries.
  ElevatorSubsystem(
      ElevatorIO io,
      double maxExtensionRotations,
      DoubleSupplier maxTravelFractionPerSecondSupplier,
      DoubleSupplier voltageSlewVoltsPerSecondSupplier) {
    this.io = io;
    this.maxExtensionRotations =
        Double.isFinite(maxExtensionRotations)
                && Math.abs(maxExtensionRotations) >= MINIMUM_VALID_CALIBRATION_ROTATIONS
            ? OptionalDouble.of(maxExtensionRotations)
            : OptionalDouble.empty();
    this.maxTravelFractionPerSecondSupplier = maxTravelFractionPerSecondSupplier;
    this.voltageSlewVoltsPerSecondSupplier = voltageSlewVoltsPerSecondSupplier;
    pollTunables();
    io.updateInputs(inputs);
    previousMotor1ExtensionFraction = motor1TravelFraction();
    previousMotor2ExtensionFraction = motor2TravelFraction();
    previousTimestampSeconds = Timer.getFPGATimestamp();
    lastMovementTimestampSeconds = previousTimestampSeconds;
  }

  public double getMotor1ExtensionRotations() {
    return -inputs.motor1Rotations;
  }

  public double getMotor2ExtensionRotations() {
    return -inputs.motor2Rotations;
  }

  /**
   * Average of both direction-corrected encoder positions — the number to read in AdvantageScope
   * when measuring the fill-in limit at full extension.
   */
  public double getExtensionRotations() {
    return (getMotor1ExtensionRotations() + getMotor2ExtensionRotations()) / 2.0;
  }

  /** True once the one limit constant holds a real measured travel; movement is refused before. */
  public boolean isCalibrated() {
    return maxExtensionRotations.isPresent();
  }

  // Travel fractions are the sign-free coordinate: each motor's extension divided by the one
  // calibrated limit. A positive and a negative measurement both normalize to 0..1 because the
  // sign cancels, so every limit check below is winding-direction agnostic.
  private double travelFraction(double extensionRotations) {
    if (!isCalibrated()) {
      return 0.0;
    }
    return extensionRotations / maxExtensionRotations.getAsDouble();
  }

  private double motor1TravelFraction() {
    return travelFraction(getMotor1ExtensionRotations());
  }

  private double motor2TravelFraction() {
    return travelFraction(getMotor2ExtensionRotations());
  }

  /** Average of both motors' fractions of full travel; above 0.5 means past the middle. */
  private double averageTravelFraction() {
    return (motor1TravelFraction() + motor2TravelFraction()) / 2.0;
  }

  /** True once either motor reaches the one shared upper limit; both motors then stop. */
  public boolean atUpperLimit() {
    return isCalibrated() && (motor1TravelFraction() >= 1.0 || motor2TravelFraction() >= 1.0);
  }

  /** True once either motor falls back to the calibrated zero; both motors then stop. */
  public boolean atLowerLimit() {
    return isCalibrated() && (motor1TravelFraction() <= 0.0 || motor2TravelFraction() <= 0.0);
  }

  /** True once either motor has extended to its share of the middle working extension. */
  public boolean atMidExtension() {
    return isCalibrated()
        && (motor1TravelFraction() >= Constants.Elevator.MID_EXTENSION_FRACTION
            || motor2TravelFraction() >= Constants.Elevator.MID_EXTENSION_FRACTION);
  }

  /** True once either motor has retracted back to its share of the middle extension. */
  public boolean atOrBelowMidExtension() {
    return isCalibrated()
        && (motor1TravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION
            || motor2TravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION);
  }

  public void extend() {
    if (!isCalibrated() || atUpperLimit()) {
      stop();
      return;
    }
    applyMovementVoltages(Constants.Elevator.MOVEMENT_VOLTAGE, true);
  }

  public void retract() {
    if (!isCalibrated() || atLowerLimit()) {
      stop();
      return;
    }
    applyMovementVoltages(Constants.Elevator.MOVEMENT_VOLTAGE, false);
  }

  /**
   * One step toward the middle working extension (button 6's destination), from whichever side
   * the elevator is on. Extension stops when either motor reaches its own share of the travel;
   * retraction from the top stops when either motor falls back to it — the same "whichever
   * mechanism arrives first" rule the hard limits use.
   */
  public void moveTowardMidExtension() {
    if (!isCalibrated()) {
      stop();
      return;
    }
    if (midMovementIsExtension) {
      if (atMidExtension() || atUpperLimit()) {
        stop();
        return;
      }
      applyMovementVoltages(Constants.Elevator.MOVEMENT_VOLTAGE, true);
    } else {
      if (atOrBelowMidExtension() || atLowerLimit()) {
        stop();
        return;
      }
      applyMovementVoltages(Constants.Elevator.MOVEMENT_VOLTAGE, false);
    }
  }

  public void stop() {
    lastMovementVoltage = 0.0;
    lastMovementTimestampSeconds = Timer.getFPGATimestamp();
    applyMotorVoltages(0.0, 0.0);
  }

  public void zeroEncoders() {
    stop();
    io.zeroEncoders();
    // Hardware takes a cycle to report the new zero; assume it immediately.
    inputs.motor1Rotations = 0.0;
    inputs.motor2Rotations = 0.0;
    previousMotor1ExtensionFraction = 0.0;
    previousMotor2ExtensionFraction = 0.0;
    motor1ExtensionFractionVelocityPerSecond = 0.0;
    motor2ExtensionFractionVelocityPerSecond = 0.0;
  }

  public Command moveToUpperLimitCommand() {
    return timedMovementCommand(
        "ElevatorToUpperLimit", this::extend, () -> !isCalibrated() || atUpperLimit());
  }

  public Command moveToLowerLimitCommand() {
    return timedMovementCommand(
        "ElevatorToLowerLimit", this::retract, () -> !isCalibrated() || atLowerLimit());
  }

  public Command moveToMidExtensionCommand() {
    return timedMovementCommand(
        "ElevatorToMidExtension",
        this::moveTowardMidExtension,
        () ->
            !isCalibrated()
                || (midMovementIsExtension
                    ? atMidExtension() || atUpperLimit()
                    : atOrBelowMidExtension() || atLowerLimit()));
  }

  /** Seconds the last movement command ran before finishing (or being interrupted), for the log. */
  public double getLastMoveDurationSeconds() {
    return lastMoveDurationSeconds;
  }

  /** Name of the last movement command that ran, e.g. "ElevatorToUpperLimit". */
  public String getLastMoveActionName() {
    return lastMoveActionName;
  }

  private Command timedMovementCommand(
      String name, Runnable moveStep, BooleanSupplier finishedCondition) {
    return new FunctionalCommand(
        () -> {
          lastMoveActionName = name;
          moveStartTimestampSeconds = Timer.getFPGATimestamp();
          // The mid command picks its direction once, at start: extending from below the
          // middle, retracting from above it.
          midMovementIsExtension =
              averageTravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION;
        },
        moveStep,
        interrupted -> {
          stop();
          lastMoveDurationSeconds = Timer.getFPGATimestamp() - moveStartTimestampSeconds;
          System.out.printf(
              "Elevator action %s took %.2f s%n", lastMoveActionName, lastMoveDurationSeconds);
        },
        finishedCondition,
        this)
        .withName(name);
  }

  /**
   * Applies the symmetric movement voltages (motor 1 and motor 2 always receive opposite signs)
   * through the spec's safety ceilings: the acceleration limit ramps the signed voltage (motor
   * 1's view; motor 2 mirrors it), so a direction reversal passes through zero instead of
   * snapping to the opposite polarity, and the speed limit cuts the voltage to zero when the
   * faster motor already travels at the cap in the commanded direction. The voltage sign follows
   * the measured calibration sign, so a rig wound the opposite way works from the same code.
   * Both ceilings are NetworkTables-tunable; 0 disables.
   */
  private void applyMovementVoltages(double targetMagnitude, boolean extending) {
    double timestampSeconds = Timer.getFPGATimestamp();
    double dtSeconds = timestampSeconds - lastMovementTimestampSeconds;
    double directionSign = Math.signum(maxExtensionRotations.getAsDouble());
    double targetMotor1Voltage =
        (extending ? targetMagnitude : -targetMagnitude) * directionSign;
    double slewedMotor1Voltage =
        slewTowards(
            targetMotor1Voltage, lastMovementVoltage, dtSeconds, voltageSlewVoltsPerSecond);
    double fasterMotorFractionPerSecond =
        extending
            ? Math.max(
                motor1ExtensionFractionVelocityPerSecond, motor2ExtensionFractionVelocityPerSecond)
            : -Math.min(
                motor1ExtensionFractionVelocityPerSecond,
                motor2ExtensionFractionVelocityPerSecond);
    double limitedMotor1Voltage =
        voltageAfterSpeedCap(
            slewedMotor1Voltage, fasterMotorFractionPerSecond, maxTravelFractionPerSecond);
    lastMovementVoltage = limitedMotor1Voltage;
    lastMovementTimestampSeconds = timestampSeconds;
    applyMotorVoltages(limitedMotor1Voltage, -limitedMotor1Voltage);
  }

  static double slewTowards(
      double targetVoltage, double currentVoltage, double dtSeconds, double voltsPerSecond) {
    if (!Double.isFinite(voltsPerSecond) || voltsPerSecond <= 0.0) {
      return targetVoltage;
    }
    if (dtSeconds <= 0.0) {
      return currentVoltage;
    }
    double maxDeltaVolts = voltsPerSecond * dtSeconds;
    return currentVoltage
        + MathUtil.clamp(targetVoltage - currentVoltage, -maxDeltaVolts, maxDeltaVolts);
  }

  static double voltageAfterSpeedCap(
      double voltage, double travelFractionPerSecondInMotionDirection, double maxFraction) {
    if (!Double.isFinite(maxFraction) || maxFraction <= 0.0) {
      return voltage;
    }
    return travelFractionPerSecondInMotionDirection >= maxFraction ? 0.0 : voltage;
  }

  private void pollTunables() {
    // One sanitized read per tuning entry per cycle. A mistyped dashboard value (non-finite
    // or negative) falls back to the validated default instead of silently disabling a
    // safety limit; zero remains a deliberate "limit off" switch.
    maxTravelFractionPerSecond =
        pollNonNegative(
            maxTravelFractionPerSecondSupplier,
            Constants.Elevator.DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND);
    voltageSlewVoltsPerSecond =
        pollNonNegative(
            voltageSlewVoltsPerSecondSupplier,
            Constants.Elevator.DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND);
  }

  private static double pollNonNegative(DoubleSupplier supplier, double fallback) {
    double value = supplier.getAsDouble();
    return Double.isFinite(value) && value >= 0.0 ? value : fallback;
  }

  private void applyMotorVoltages(double motor1Voltage, double motor2Voltage) {
    io.setMotorVoltages(motor1Voltage, motor2Voltage);
  }

  @Override
  public void periodic() {
    // Poll the tuning entries first: one sanitized read per entry per cycle, shared by every
    // consumer below (the same once-per-cycle pattern the sensor inputs follow).
    pollTunables();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    double timestampSeconds = Timer.getFPGATimestamp();
    double dtSeconds = timestampSeconds - previousTimestampSeconds;
    if (dtSeconds > MINIMUM_VELOCITY_TIMESTEP_SECONDS) {
      double motor1RawVelocity =
          (motor1TravelFraction() - previousMotor1ExtensionFraction) / dtSeconds;
      double motor2RawVelocity =
          (motor2TravelFraction() - previousMotor2ExtensionFraction) / dtSeconds;
      // Low-pass the finite-difference velocities so encoder quantization cannot trip the
      // speed ceiling with per-cycle spikes (same filter ArmDemo uses).
      motor1ExtensionFractionVelocityPerSecond =
          VELOCITY_FILTER_ALPHA * motor1RawVelocity
              + (1.0 - VELOCITY_FILTER_ALPHA) * motor1ExtensionFractionVelocityPerSecond;
      motor2ExtensionFractionVelocityPerSecond =
          VELOCITY_FILTER_ALPHA * motor2RawVelocity
              + (1.0 - VELOCITY_FILTER_ALPHA) * motor2ExtensionFractionVelocityPerSecond;
      previousMotor1ExtensionFraction = motor1TravelFraction();
      previousMotor2ExtensionFraction = motor2TravelFraction();
      previousTimestampSeconds = timestampSeconds;
    }

    Logger.recordOutput(
        "Elevator/Motor1ExtensionRotations", getMotor1ExtensionRotations(), "rotations");
    Logger.recordOutput(
        "Elevator/Motor2ExtensionRotations", getMotor2ExtensionRotations(), "rotations");
    Logger.recordOutput("Elevator/ExtensionRotations", getExtensionRotations(), "rotations");
    Logger.recordOutput("Elevator/RawMotor1Rotations", inputs.motor1Rotations, "rotations");
    Logger.recordOutput("Elevator/RawMotor2Rotations", inputs.motor2Rotations, "rotations");
    Logger.recordOutput("Elevator/ExtensionFraction", averageTravelFraction());
    Logger.recordOutput("Elevator/Motor1AppliedVoltage", inputs.motor1AppliedVolts, "volts");
    Logger.recordOutput("Elevator/Motor2AppliedVoltage", inputs.motor2AppliedVolts, "volts");
    Logger.recordOutput("Elevator/AtLowerLimit", atLowerLimit());
    Logger.recordOutput("Elevator/AtUpperLimit", atUpperLimit());
    Logger.recordOutput("Elevator/Calibrated", isCalibrated());
    Logger.recordOutput(
        "Elevator/MaxExtensionRotations", maxExtensionRotations.orElse(0.0), "rotations");
    Logger.recordOutput(
        "Elevator/Motor1TravelFractionPerSecond",
        motor1ExtensionFractionVelocityPerSecond,
        "x/s");
    Logger.recordOutput(
        "Elevator/Motor2TravelFractionPerSecond",
        motor2ExtensionFractionVelocityPerSecond,
        "x/s");
    Logger.recordOutput("Elevator/LastMoveDurationSeconds", lastMoveDurationSeconds, "seconds");
    Logger.recordOutput("Elevator/LastMoveActionName", lastMoveActionName);
  }
}
