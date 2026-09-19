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
import frc.robot.ElevatorCalibration;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Dual-motor elevator: open-loop voltage motion bounded by software travel limits, plus an optional
 * position controller for presets.
 *
 * <p><b>One owner of the output.</b> Every voltage is written from {@link #periodic()}: commands
 * only request a {@link MotionMode} and read back whether it completed. That ordering matters more
 * than it looks — {@code CommandScheduler} runs a command's {@code execute()} <i>before</i> the
 * subsystem's {@code periodic()} in the same cycle, so a limit test performed inside {@code
 * execute()} acts on a position snapshot from 20 ms ago, and the last voltage it approves is on the
 * wires for a whole cycle after the mechanism has already reached its stop. Here the freshest
 * snapshot of the cycle is polled and then the limits are applied against it, so a limit that
 * triggers stops the output in the same cycle it was seen.
 *
 * <p><b>Per-motor travels.</b> Each motor's position is normalized by its own calibrated travel
 * (or by the one shared travel on a symmetric rig), so both motors' fractions reach 1.0 at the same
 * physical height. Dividing both by a single averaged number would trip "whichever motor arrives
 * first" at a fraction of the real travel on any geared rig — see {@link ElevatorCalibration} and
 * docs/CALIBRATION.md.
 *
 * <p><b>Signs are explicit.</b> Whether the rotor reads positive or negative while the elevator
 * extends, and which voltage polarity extends it, are two separate physical facts
 * ({@link Constants.Elevator#EXTENSION_ROTOR_SIGN}, {@link
 * Constants.Elevator#EXTENSION_VOLTAGE_SIGN}). Calibration travels are magnitudes. A mechanism that
 * turns out to be wired the other way shows up as a contradirectional reading, which
 * {@link #periodic()} detects and latches as a fault instead of driving away from its limits.
 */
public class ElevatorSubsystem extends SubsystemBase {
  /** What the elevator is being told to do. Only {@link #periodic()} turns this into volts. */
  public enum MotionMode {
    STOPPED,
    EXTENDING,
    RETRACTING,
    MOVING_TO_MID,
    MOVING_TO_FRACTION
  }

  private static final double MINIMUM_VELOCITY_TIMESTEP_SECONDS = 1e-6;
  private static final double VELOCITY_FILTER_ALPHA = 0.3;
  // Direction monitoring: enough applied voltage to expect motion, how long to wait before
  // judging, and how far the mechanism must have moved before the judgement means anything.
  private static final double MINIMUM_VOLTAGE_TO_JUDGE_DIRECTION = 0.5;
  private static final double DIRECTION_JUDGEMENT_SETTLE_SECONDS = 0.15;
  private static final double DIRECTION_JUDGEMENT_MIN_TRAVEL_FRACTION = 0.02;
  // Below this much of the travel in use, the two motors' readings are too close to compare for a
  // gearing-ratio sanity check to mean anything.
  private static final double MINIMUM_FRACTION_FOR_RATIO_DIAGNOSTIC = 0.2;

  private final ElevatorIO io;
  // Sensor inputs: filled by io.updateInputs() once per cycle, logged wholesale for replay.
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorCalibration calibration;
  private final Tunables tunables;
  // Time source: the real clock in production, a manual one in tests, so nothing has to sleep to
  // prove that a ramp or an arrival window behaves.
  private final DoubleSupplier clockSeconds;

  // Sanitized snapshots of the tunables, refreshed once per cycle and shared by every consumer.
  private double maxTravelFractionPerSecond;
  private double voltageSlewVoltsPerSecond;
  private double sensorTimeoutMilliseconds;
  private double positionKpVoltsPerTravel;
  private double positionArriveToleranceFraction;
  private double gravityFeedforwardVolts;

  // Finite-difference velocity estimate, low-passed, in fractions of travel per second.
  private double previousMotor1ExtensionFraction;
  private double previousMotor2ExtensionFraction;
  private double previousTimestampSeconds;
  private double motor1ExtensionFractionVelocityPerSecond;
  private double motor2ExtensionFractionVelocityPerSecond;

  // Signed motor-1 voltage actually applied last cycle, plus when. The ramp works on this signed
  // value so a reversal passes through zero instead of snapping to the opposite polarity.
  private double lastMovementVoltage;
  private double lastMovementTimestampSeconds;

  private MotionMode mode = MotionMode.STOPPED;
  private double targetExtensionFraction;
  private boolean midMovementIsExtension;
  private boolean positionIsArrived;

  // Records kept for the spec's "every action has a completion time".
  private double moveStartTimestampSeconds;
  private double lastMoveDurationSeconds = -1.0;
  private String lastMoveActionName = "";

  // Direction fault: latched when the mechanism demonstrably moves against what was commanded.
  private double judgementStartTimestampSeconds = Double.NaN;
  private double judgementStartFraction;
  private boolean directionIsReversed;
  private boolean motionWasDetected;
  private double measuredTravelRatio;
  private boolean calibrationIsSuspect;

  /**
   * Production constructor. The wiring passes the calibration in explicitly (see {@link
   * frc.robot.RobotContainer}) so the one value that gates every movement is visible at the point of
   * assembly, while all of the operator-adjustable ceilings stay owned by this class.
   */
  public ElevatorSubsystem(ElevatorIO io, ElevatorCalibration calibration) {
    this(io, calibration, Tunables.fromNetworkTables(), Timer::getFPGATimestamp);
  }

  /** Test seam: pin the calibration, every ceiling, and the clock. */
  ElevatorSubsystem(
      ElevatorIO io,
      ElevatorCalibration calibration,
      Tunables tunables,
      DoubleSupplier clockSeconds) {
    this.io = io;
    this.calibration = calibration;
    this.tunables = tunables;
    this.clockSeconds = clockSeconds;
    refreshTunables();
    io.updateInputs(inputs);
    previousMotor1ExtensionFraction = motor1TravelFraction();
    previousMotor2ExtensionFraction = motor2TravelFraction();
    previousTimestampSeconds = now();
    lastMovementTimestampSeconds = previousTimestampSeconds;
  }

  private double now() {
    return clockSeconds.getAsDouble();
  }

  // ---------------------------------------------------------------------------
  // Position reads

  /** Motor 1 position in the extension-positive convention. */
  public double getMotor1ExtensionRotations() {
    return inputs.motor1Rotations * Constants.Elevator.EXTENSION_ROTOR_SIGN;
  }

  /** Motor 2 position in the extension-positive convention. */
  public double getMotor2ExtensionRotations() {
    return inputs.motor2Rotations * Constants.Elevator.EXTENSION_ROTOR_SIGN;
  }

  /**
   * Average of both direction-corrected positions. On a symmetric rig this is the number to read
   * when filling in the one shared travel; on a geared rig read both motors separately instead
   * (docs/CALIBRATION.md).
   */
  public double getExtensionRotations() {
    return (getMotor1ExtensionRotations() + getMotor2ExtensionRotations()) / 2.0;
  }

  /** True once a usable travel exists; every movement is refused before that. */
  public boolean isCalibrated() {
    return calibration.isCalibrated();
  }

  /** Motor 1's position as a fraction of its own calibrated travel. */
  private double travelFraction(int motor, double extensionRotations) {
    return isCalibrated() ? calibration.travelFraction(motor, extensionRotations) : 0.0;
  }

  private double motor1TravelFraction() {
    return travelFraction(1, getMotor1ExtensionRotations());
  }

  private double motor2TravelFraction() {
    return travelFraction(2, getMotor2ExtensionRotations());
  }

  /** Average of both motors' fractions of their own travel; 0 down, 1 fully extended. */
  public double getExtensionFraction() {
    return averageTravelFraction();
  }

  private double averageTravelFraction() {
    return (motor1TravelFraction() + motor2TravelFraction()) / 2.0;
  }

  // ---------------------------------------------------------------------------
  // Limits and permission to drive

  /** True once either motor reaches its own share of full travel; both motors then stop. */
  public boolean atUpperLimit() {
    return isCalibrated() && (motor1TravelFraction() >= 1.0 || motor2TravelFraction() >= 1.0);
  }

  /** True once either motor falls back to the calibrated zero; both motors then stop. */
  public boolean atLowerLimit() {
    return isCalibrated() && (motor1TravelFraction() <= 0.0 || motor2TravelFraction() <= 0.0);
  }

  /** True once either motor has extended to its own share of the middle working extension. */
  public boolean atMidExtension() {
    return isCalibrated()
        && (motor1TravelFraction() >= Constants.Elevator.MID_EXTENSION_FRACTION
            || motor2TravelFraction() >= Constants.Elevator.MID_EXTENSION_FRACTION);
  }

  /** True once either motor has retracted back to its own share of the middle extension. */
  public boolean atOrBelowMidExtension() {
    return isCalibrated()
        && (motor1TravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION
            || motor2TravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION);
  }

  /**
   * True while both drives have answered cleanly and the age check passes. A frozen position is
   * worse than no position, because it keeps reading "not at the limit"; untrusted sensors mean the
   * output is cut regardless of the requested mode.
   */
  public boolean sensorsAreTrusted() {
    return positionIsTrusted(inputs.motor1PositionIsResponding, inputs.motor1PositionAgeMilliseconds)
        && positionIsTrusted(inputs.motor2PositionIsResponding, inputs.motor2PositionAgeMilliseconds);
  }

  private boolean positionIsTrusted(boolean isResponding, double ageMilliseconds) {
    if (!isResponding) {
      return false;
    }
    // A timeout of 0 disables only the age test; the transaction checks above always apply.
    return sensorTimeoutMilliseconds <= 0.0 || ageMilliseconds <= sensorTimeoutMilliseconds;
  }

  /** True when driving is permitted at all: calibrated, sensors trusted, and no direction fault. */
  public boolean canDrive() {
    return isCalibrated() && sensorsAreTrusted() && !directionIsReversed;
  }

  // ---------------------------------------------------------------------------
  // Mode requests

  /** Requests a mode, restarting the direction monitor's observation window. */
  private void requestMode(MotionMode requested) {
    if (mode == requested) {
      return;
    }
    mode = requested;
    judgementStartTimestampSeconds = Double.NaN;
    positionIsArrived = false;
    if (requested == MotionMode.MOVING_TO_MID) {
      // Latched once, at the start: extend from below the middle, retract from above it.
      midMovementIsExtension =
          averageTravelFraction() <= Constants.Elevator.MID_EXTENSION_FRACTION;
    }
  }

  /**
   * Declares the current position the calibration zero, which is only a physically meaningful claim
   * while the mechanism rests on its lower stop. Away from the stop this returns false and changes
   * nothing, because zeroing mid-travel would silently move every software limit by that amount —
   * the exact failure this project's safety model cannot tolerate.
   */
  public boolean rezeroAtLowerLimit() {
    if (!isCalibrated() || !sensorsAreTrusted() || !atLowerLimit()) {
      return false;
    }
    stop();
    io.zeroEncoders();
    // Hardware takes a cycle to report the new zero; assume it immediately.
    inputs.motor1Rotations = 0.0;
    inputs.motor2Rotations = 0.0;
    previousMotor1ExtensionFraction = 0.0;
    previousMotor2ExtensionFraction = 0.0;
    motor1ExtensionFractionVelocityPerSecond = 0.0;
    motor2ExtensionFractionVelocityPerSecond = 0.0;
    measuredTravelRatio = 0.0;
    calibrationIsSuspect = false;
    return true;
  }

  /** Releases the output and stops the direction monitor. */
  public void stop() {
    lastMovementVoltage = 0.0;
    lastMovementTimestampSeconds = now();
    mode = MotionMode.STOPPED;
    judgementStartTimestampSeconds = Double.NaN;
    // positionIsArrived deliberately survives: it records whether the last position request
    // landed, which is what the log and the tests read back after the request has ended.
    io.setMotorVoltages(0.0, 0.0);
  }

  /** The latched direction fault, which only clearing it (or a restart) can release. */
  public boolean directionIsReversed() {
    return directionIsReversed;
  }

  /**
   * True once the two motors' live readings have proven further apart than the one shared travel
   * they are being normalized by allows — the signature of a geared rig calibrated the way this
   * project originally did. Fix it by filling both per-motor travels.
   */
  public boolean calibrationIsSuspect() {
    return calibrationIsSuspect;
  }

  public MotionMode getMode() {
    return mode;
  }

  public double getTargetExtensionFraction() {
    return targetExtensionFraction;
  }

  // ---------------------------------------------------------------------------
  // Commands

  public Command moveToUpperLimitCommand() {
    return modeCommand("ElevatorToUpperLimit", MotionMode.EXTENDING, () -> !canDrive() || atUpperLimit());
  }

  public Command moveToLowerLimitCommand() {
    return modeCommand("ElevatorToLowerLimit", MotionMode.RETRACTING, () -> !canDrive() || atLowerLimit());
  }

  public Command moveToMidExtensionCommand() {
    return modeCommand(
        "ElevatorToMidExtension",
        MotionMode.MOVING_TO_MID,
        () ->
            !canDrive()
                || (midMovementIsExtension
                    ? atMidExtension() || atUpperLimit()
                    : atOrBelowMidExtension() || atLowerLimit()));
  }

  /**
   * Position-controlled move to a fraction of the calibrated travel; finishes once inside the
   * arrive window. The preset stays inside the travel by construction, and an uncalibrated or
   * untrusted elevator ends it immediately at zero volts.
   */
  public Command moveToFractionCommand(double fraction) {
    return positionCommand("ElevatorToFraction", fraction, false);
  }

  /**
   * Same move, but the request never finishes on its own: it holds the position (proportional term
   * plus the gravity feedforward) until something interrupts it. Useful while a mechanism is being
   * loaded, and the shape a real scoring command takes on a competition robot.
   */
  public Command holdAtFractionCommand(double fraction) {
    return positionCommand("ElevatorHoldFraction", fraction, true);
  }

  private Command positionCommand(String name, double fraction, boolean hold) {
    return new FunctionalCommand(
            () -> {
              targetExtensionFraction = MathUtil.clamp(fraction, 0.0, 1.0);
              requestMode(MotionMode.MOVING_TO_FRACTION);
              beginAction(name);
            },
            () -> {},
            interrupted -> endAction(),
            () -> !canDrive() || (positionIsArrived && !hold),
            this)
        .withName(name);
  }

  /**
   * Re-declares the calibration zero, and only while the mechanism is resting on its lower stop.
   * Bound to a button so a mid-match manual move can be forgiven without a power cycle.
   */
  public Command rezeroAtLowerLimitCommand() {
    return runOnce(
            () -> {
              boolean accepted = rezeroAtLowerLimit();
              Logger.recordOutput("Elevator/RezeroAccepted", accepted);
              if (!accepted) {
                System.out.println("Elevator re-zero refused: not resting on the lower limit");
              }
            })
        .withName("ElevatorRezeroAtLowerLimit");
  }

  /** Seconds the last movement command ran, including one interrupted by another command. */
  public double getLastMoveDurationSeconds() {
    return lastMoveDurationSeconds;
  }

  /** Name of the last movement command that ran, e.g. "ElevatorToUpperLimit". */
  public String getLastMoveActionName() {
    return lastMoveActionName;
  }

  private Command modeCommand(String name, MotionMode requested, BooleanSupplier finished) {
    return new FunctionalCommand(
            () -> {
              requestMode(requested);
              beginAction(name);
            },
            () -> {},
            interrupted -> endAction(),
            finished,
            this)
        .withName(name);
  }

  private void beginAction(String name) {
    lastMoveActionName = name;
    moveStartTimestampSeconds = now();
  }

  private void endAction() {
    stop();
    lastMoveDurationSeconds = now() - moveStartTimestampSeconds;
    System.out.printf(
        "Elevator action %s took %.2f s%n", lastMoveActionName, lastMoveDurationSeconds);
  }

  // ---------------------------------------------------------------------------
  // Control loop

  @Override
  public void periodic() {
    // Poll the tunables first so every consumer below, including the sensor-age policy, sees the
    // same one sanitized snapshot of this cycle's values.
    refreshTunables();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    double timestampSeconds = now();
    updateMotionEstimate(timestampSeconds);
    enforceRequestedMotion(timestampSeconds);
    updateDiagnostics();
    logOutputs();
  }

  /** Turns the requested mode into volts, bounded by the limits and the spec's safety ceilings. */
  private void enforceRequestedMotion(double timestampSeconds) {
    if (!canDrive()) {
      // Uncalibrated, sensors untrustworthy, or a latched direction fault: nothing moves.
      applyMovementVoltage(0.0, timestampSeconds);
      return;
    }
    double movementVoltage =
        switch (mode) {
          case STOPPED -> 0.0;
          case EXTENDING -> atUpperLimit() ? 0.0 : extensionVoltage();
          case RETRACTING -> atLowerLimit() ? 0.0 : -extensionVoltage();
          case MOVING_TO_MID -> midMovementVoltage();
          case MOVING_TO_FRACTION -> positionVoltage();
        };
    applyMovementVoltage(movementVoltage, timestampSeconds);
    updateDirectionMonitor(timestampSeconds, movementVoltage);
  }

  private double extensionVoltage() {
    return Constants.Elevator.MOVEMENT_VOLTAGE * extensionDirectionSign();
  }

  /**
   * +1 or -1: the sign of the motor-1 voltage that extends the elevator. Every direction decision
   * below compares against this rather than assuming a positive voltage means extension.
   */
  private static double extensionDirectionSign() {
    return Constants.Elevator.EXTENSION_VOLTAGE_SIGN >= 0.0 ? 1.0 : -1.0;
  }

  /** A magnitude expressed in the extending direction, for the gravity feedforward term. */
  private static double extensionVolts(double magnitude) {
    return magnitude * extensionDirectionSign();
  }

  private double midMovementVoltage() {
    if (midMovementIsExtension) {
      return atMidExtension() || atUpperLimit() ? 0.0 : extensionVoltage();
    }
    return atOrBelowMidExtension() || atLowerLimit() ? 0.0 : -extensionVoltage();
  }

  /**
   * Proportional position control in fraction space, so it inherits the per-motor normalization and
   * needs no stage ratio. The output is capped at the same fixed movement voltage the limit-style
   * moves use, then passes through the shared ramp and speed ceiling. While the mechanism is
   * stationary at its target the gravity feedforward is what stays there, and it starts at 0 V on
   * purpose: this rig's holding voltage has not been measured, and a guessed number is worse than a
   * documented blank.
   */
  private double positionVoltage() {
    double errorFraction = targetExtensionFraction - averageTravelFraction();
    if (Math.abs(errorFraction) <= positionArriveToleranceFraction) {
      positionIsArrived = true;
      return extensionVolts(gravityFeedforwardVolts);
    }
    double cap = Math.abs(extensionVoltage());
    double commandVolts =
        MathUtil.clamp(positionKpVoltsPerTravel * errorFraction, -cap, cap)
            + extensionVolts(gravityFeedforwardVolts);
    // Never push past a hard limit on the way to a preset, whatever the controller asks for.
    boolean pushingUpward = commandVolts * extensionDirectionSign() > 0.0;
    if (pushingUpward && atUpperLimit()) {
      return extensionVolts(gravityFeedforwardVolts);
    }
    if (!pushingUpward && atLowerLimit()) {
      return 0.0;
    }
    return commandVolts;
  }

  /**
   * Watches for the mechanism demonstrably moving against the command, which is how a wrong sign
   * constant or a swapped pair of CAN IDs shows up. Only sustained contradirectional travel trips
   * it: a mechanism that refuses to move at all could simply be braked or stalled, which is reported
   * as telemetry rather than latched into a fault.
   */
  private void updateDirectionMonitor(double timestampSeconds, double movementVoltage) {
    if (Math.abs(movementVoltage) < MINIMUM_VOLTAGE_TO_JUDGE_DIRECTION) {
      judgementStartTimestampSeconds = Double.NaN;
      return;
    }
    boolean extending = movementVoltage * extensionDirectionSign() > 0.0;
    double currentFraction = averageTravelFraction();
    if (Double.isNaN(judgementStartTimestampSeconds)) {
      judgementStartTimestampSeconds = timestampSeconds;
      judgementStartFraction = currentFraction;
      return;
    }
    if (timestampSeconds - judgementStartTimestampSeconds < DIRECTION_JUDGEMENT_SETTLE_SECONDS) {
      return;
    }
    double travelled = currentFraction - judgementStartFraction;
    if (Math.abs(travelled) < DIRECTION_JUDGEMENT_MIN_TRAVEL_FRACTION) {
      return;
    }
    motionWasDetected = true;
    boolean movedTheWrongWay = extending ? travelled < 0.0 : travelled > 0.0;
    if (movedTheWrongWay && !directionIsReversed) {
      directionIsReversed = true;
      System.err.println(
          "Elevator direction fault: the mechanism moved against the command. It will not drive"
              + " until the EXTENSION_ROTOR_SIGN / EXTENSION_VOLTAGE_SIGN constants match the"
              + " wiring (see docs/CALIBRATION.md).");
    }
  }

  /**
   * Compares the two motors' live readings. On a symmetric rig they must track each other; when one
   * clearly covers more travel than the other, one shared limit is the wrong model, which is exactly
   * the mistake this project started out making.
   */
  private void updateDiagnostics() {
    if (!isCalibrated() || !sensorsAreTrusted()) {
      return;
    }
    double motor1Rotations = getMotor1ExtensionRotations();
    double motor2Rotations = getMotor2ExtensionRotations();
    double smaller = Math.min(Math.abs(motor1Rotations), Math.abs(motor2Rotations));
    boolean farEnoughApartToCompare =
        Math.abs(motor1TravelFraction()) >= MINIMUM_FRACTION_FOR_RATIO_DIAGNOSTIC
            || Math.abs(motor2TravelFraction()) >= MINIMUM_FRACTION_FOR_RATIO_DIAGNOSTIC;
    if (!farEnoughApartToCompare || smaller < 1.0e-6) {
      return;
    }
    measuredTravelRatio =
        Math.max(Math.abs(motor1Rotations), Math.abs(motor2Rotations)) / smaller;
    if (calibration.reliesOnSharedTravelAssumption()
        && measuredTravelRatio > Constants.Elevator.TRAVEL_ASYMMETRY_WARNING_RATIO) {
      calibrationIsSuspect = true;
    }
  }

  /** Applies a motor-1 voltage through the acceleration ramp and the speed ceiling. */
  private void applyMovementVoltage(double targetMotor1Voltage, double timestampSeconds) {
    double dtSeconds = timestampSeconds - lastMovementTimestampSeconds;
    double slewedMotor1Voltage =
        slewTowards(
            targetMotor1Voltage, lastMovementVoltage, dtSeconds, voltageSlewVoltsPerSecond);
    double rateInVoltageDirection =
        slewedMotor1Voltage >= 0.0
            ? Math.max(
                motor1ExtensionFractionVelocityPerSecond,
                motor2ExtensionFractionVelocityPerSecond)
            : -Math.min(
                motor1ExtensionFractionVelocityPerSecond,
                motor2ExtensionFractionVelocityPerSecond);
    double limitedMotor1Voltage =
        voltageAfterSpeedCap(slewedMotor1Voltage, rateInVoltageDirection, maxTravelFractionPerSecond);
    lastMovementVoltage = limitedMotor1Voltage;
    lastMovementTimestampSeconds = timestampSeconds;
    io.setMotorVoltages(limitedMotor1Voltage, -limitedMotor1Voltage);
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

  private void updateMotionEstimate(double timestampSeconds) {
    double dtSeconds = timestampSeconds - previousTimestampSeconds;
    if (dtSeconds <= MINIMUM_VELOCITY_TIMESTEP_SECONDS) {
      return;
    }
    double currentMotor1Fraction = motor1TravelFraction();
    double currentMotor2Fraction = motor2TravelFraction();
    double motor1RawVelocity = (currentMotor1Fraction - previousMotor1ExtensionFraction) / dtSeconds;
    double motor2RawVelocity = (currentMotor2Fraction - previousMotor2ExtensionFraction) / dtSeconds;
    // Low-pass the finite-difference velocities so encoder quantization cannot trip the speed
    // ceiling with per-cycle spikes (same filter ArmDemo uses).
    motor1ExtensionFractionVelocityPerSecond =
        VELOCITY_FILTER_ALPHA * motor1RawVelocity
            + (1.0 - VELOCITY_FILTER_ALPHA) * motor1ExtensionFractionVelocityPerSecond;
    motor2ExtensionFractionVelocityPerSecond =
        VELOCITY_FILTER_ALPHA * motor2RawVelocity
            + (1.0 - VELOCITY_FILTER_ALPHA) * motor2ExtensionFractionVelocityPerSecond;
    previousMotor1ExtensionFraction = currentMotor1Fraction;
    previousMotor2ExtensionFraction = currentMotor2Fraction;
    previousTimestampSeconds = timestampSeconds;
  }

  private void refreshTunables() {
    maxTravelFractionPerSecond =
        tunables.nonNegative(
            tunables.maxTravelFractionPerSecond,
            Constants.Elevator.DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND);
    voltageSlewVoltsPerSecond =
        tunables.nonNegative(
            tunables.voltageSlewVoltsPerSecond,
            Constants.Elevator.DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND);
    sensorTimeoutMilliseconds =
        tunables.nonNegative(
            tunables.sensorTimeoutMilliseconds,
            Constants.Elevator.DEFAULT_SENSOR_TIMEOUT_MILLISECONDS);
    positionKpVoltsPerTravel =
        tunables.nonNegative(
            tunables.positionKpVoltsPerTravel,
            Constants.Elevator.DEFAULT_POSITION_KP_VOLTS_PER_TRAVEL);
    positionArriveToleranceFraction =
        tunables.nonNegative(
            tunables.positionArriveToleranceFraction,
            Constants.Elevator.DEFAULT_POSITION_ARRIVE_TOLERANCE_FRACTION);
    gravityFeedforwardVolts =
        tunables.nonNegative(
            tunables.gravityFeedforwardVolts, Constants.Elevator.DEFAULT_GRAVITY_FEEDFORWARD_VOLTS);
  }

  private void logOutputs() {
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
    Logger.recordOutput("Elevator/MaxExtensionRotations", motor1TravelRotationsForLog(), "rotations");
    Logger.recordOutput(
        "Elevator/Motor2MaxExtensionRotations", motor2TravelRotationsForLog(), "rotations");
    Logger.recordOutput("Elevator/CalibrationSource", calibration.source().name());
    Logger.recordOutput("Elevator/ConfiguredTravelRatio", calibration.configuredTravelRatio());
    Logger.recordOutput("Elevator/MeasuredTravelRatio", measuredTravelRatio);
    Logger.recordOutput("Elevator/CalibrationSuspect", calibrationIsSuspect);
    Logger.recordOutput(
        "Elevator/Motor1PositionAgeMilliseconds",
        finiteOrNegativeOne(inputs.motor1PositionAgeMilliseconds),
        "milliseconds");
    Logger.recordOutput(
        "Elevator/Motor2PositionAgeMilliseconds",
        finiteOrNegativeOne(inputs.motor2PositionAgeMilliseconds),
        "milliseconds");
    Logger.recordOutput("Elevator/SensorsTrusted", sensorsAreTrusted());
    Logger.recordOutput("Elevator/Mode", mode.name());
    Logger.recordOutput("Elevator/TargetExtensionFraction", targetExtensionFraction);
    Logger.recordOutput(
        "Elevator/PositionErrorFraction", targetExtensionFraction - averageTravelFraction());
    Logger.recordOutput("Elevator/PositionArrived", positionIsArrived);
    Logger.recordOutput("Elevator/DirectionReversed", directionIsReversed);
    Logger.recordOutput("Elevator/MotionDetected", motionWasDetected);
    Logger.recordOutput(
        "Elevator/Motor1TravelFractionPerSecond", motor1ExtensionFractionVelocityPerSecond, "x/s");
    Logger.recordOutput(
        "Elevator/Motor2TravelFractionPerSecond", motor2ExtensionFractionVelocityPerSecond, "x/s");
    Logger.recordOutput("Elevator/LastMoveDurationSeconds", lastMoveDurationSeconds, "seconds");
    Logger.recordOutput("Elevator/LastMoveActionName", lastMoveActionName);
  }

  private double motor1TravelRotationsForLog() {
    return isCalibrated() ? calibration.motor1TravelRotations() : 0.0;
  }

  private double motor2TravelRotationsForLog() {
    return isCalibrated() ? calibration.motor2TravelRotations() : 0.0;
  }

  /**
   * NetworkTables cannot carry an infinite age (the "this drive never reported" case), so it is
   * published as -1 and documented as such.
   */
  private static double finiteOrNegativeOne(double value) {
    return Double.isFinite(value) ? value : -1.0;
  }

  /**
   * Live-tunable sources, owned by the subsystem. Production reads NetworkTables; tests hand in
   * constants or mutable suppliers.
   */
  static class Tunables {
    final DoubleSupplier maxTravelFractionPerSecond;
    final DoubleSupplier voltageSlewVoltsPerSecond;
    final DoubleSupplier sensorTimeoutMilliseconds;
    final DoubleSupplier positionKpVoltsPerTravel;
    final DoubleSupplier positionArriveToleranceFraction;
    final DoubleSupplier gravityFeedforwardVolts;

    Tunables(
        DoubleSupplier maxTravelFractionPerSecond,
        DoubleSupplier voltageSlewVoltsPerSecond,
        DoubleSupplier sensorTimeoutMilliseconds,
        DoubleSupplier positionKpVoltsPerTravel,
        DoubleSupplier positionArriveToleranceFraction,
        DoubleSupplier gravityFeedforwardVolts) {
      this.maxTravelFractionPerSecond = maxTravelFractionPerSecond;
      this.voltageSlewVoltsPerSecond = voltageSlewVoltsPerSecond;
      this.sensorTimeoutMilliseconds = sensorTimeoutMilliseconds;
      this.positionKpVoltsPerTravel = positionKpVoltsPerTravel;
      this.positionArriveToleranceFraction = positionArriveToleranceFraction;
      this.gravityFeedforwardVolts = gravityFeedforwardVolts;
    }

    /** Every entry the operator can move mid-match, with its validated default. */
    static Tunables fromNetworkTables() {
      return new Tunables(
          networkNumber("/SmartDashboard/Elevator Max Travel (/s)",
              Constants.Elevator.DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND),
          networkNumber("/SmartDashboard/Elevator Voltage Slew (V/s)",
              Constants.Elevator.DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND),
          networkNumber("/SmartDashboard/Elevator Sensor Timeout (ms)",
              Constants.Elevator.DEFAULT_SENSOR_TIMEOUT_MILLISECONDS),
          networkNumber("/SmartDashboard/Elevator Position kP (V/travel)",
              Constants.Elevator.DEFAULT_POSITION_KP_VOLTS_PER_TRAVEL),
          networkNumber("/SmartDashboard/Elevator Arrive Tolerance (travel)",
              Constants.Elevator.DEFAULT_POSITION_ARRIVE_TOLERANCE_FRACTION),
          networkNumber("/SmartDashboard/Elevator Gravity Feedforward (V)",
              Constants.Elevator.DEFAULT_GRAVITY_FEEDFORWARD_VOLTS));
    }

    /**
     * Both motion ceilings off and the sensor age budget at its declared default (0 here would mean
     * "no age limit", which is a different switch). Gains sit at their documented defaults.
     */
    static Tunables ceilingsOff() {
      return new Tunables(
          () -> 0.0,
          () -> 0.0,
          () -> Constants.Elevator.DEFAULT_SENSOR_TIMEOUT_MILLISECONDS,
          () -> Constants.Elevator.DEFAULT_POSITION_KP_VOLTS_PER_TRAVEL,
          () -> Constants.Elevator.DEFAULT_POSITION_ARRIVE_TOLERANCE_FRACTION,
          () -> Constants.Elevator.DEFAULT_GRAVITY_FEEDFORWARD_VOLTS);
    }

    static Tunables withSlew(DoubleSupplier slewVoltsPerSecond) {
      return new Tunables(
          () -> 0.0,
          slewVoltsPerSecond,
          () -> Constants.Elevator.DEFAULT_SENSOR_TIMEOUT_MILLISECONDS,
          () -> Constants.Elevator.DEFAULT_POSITION_KP_VOLTS_PER_TRAVEL,
          () -> Constants.Elevator.DEFAULT_POSITION_ARRIVE_TOLERANCE_FRACTION,
          () -> Constants.Elevator.DEFAULT_GRAVITY_FEEDFORWARD_VOLTS);
    }

    static Tunables withPositionGains(double kpVoltsPerTravel, double arriveToleranceFraction) {
      return new Tunables(
          () -> 0.0,
          () -> 0.0,
          () -> Constants.Elevator.DEFAULT_SENSOR_TIMEOUT_MILLISECONDS,
          () -> kpVoltsPerTravel,
          () -> arriveToleranceFraction,
          () -> Constants.Elevator.DEFAULT_GRAVITY_FEEDFORWARD_VOLTS);
    }

    private static DoubleSupplier networkNumber(String key, double defaultValue) {
      LoggedNetworkNumber entry = new LoggedNetworkNumber(key, defaultValue);
      return entry::get;
    }

    /**
     * One sanitized read per cycle: a mistyped dashboard value (non-finite or negative) falls back
     * to the validated default instead of silently disabling a safety limit, while 0 remains the
     * deliberate "this ceiling is off" switch.
     */
    double nonNegative(DoubleSupplier supplier, double fallback) {
      double value = supplier.getAsDouble();
      return Double.isFinite(value) && value >= 0.0 ? value : fallback;
    }
  }
}
