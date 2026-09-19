// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.OptionalDouble;

/**
 * The elevator's resolved travel calibration: how many direction-corrected motor rotations it
 * takes each motor to go from the calibrated zero to full extension.
 *
 * <p>Two rigs are supported, and the difference is the whole reason this type exists. When both
 * motors turn the same amount the calibration is a single shared number. When they are geared
 * differently — ArmDemo measured 3.4 rotations on one motor and 14.5 on the other for the same
 * bench elevator — one shared number is wrong: every limit check that uses "whichever mechanism
 * arrives first" would trip on the fast motor at a fraction of the real travel and the elevator
 * would quietly never reach the top. This class keeps per-motor travels as the coordinate, so
 * both motors' fractions reach 1.0 at the same physical height on either rig.
 *
 * <p>Travels are positive magnitudes. Which way the raw encoder turns and which polarity extends
 * the mechanism are separate physical facts, held in their own constants
 * ({@link Constants.Elevator#EXTENSION_ROTOR_SIGN}, {@link
 * Constants.Elevator#EXTENSION_VOLTAGE_SIGN}) and applied before positions ever reach here — so a
 * rig wound the other way changes those two signs, not the calibration numbers, and nothing has to
 * guess which convention a signed travel was meant to encode.
 */
public final class ElevatorCalibration {
  /** How the configured constants resolved. */
  public enum Source {
    /** Both motors have their own travel. */
    PER_MOTOR,
    /** One travel serves both motors. */
    SHARED,
    /** Nothing usable was configured, so movement is refused. */
    UNCALIBRATED
  }

  private static final double MINIMUM_VALID_TRAVEL_ROTATIONS = 1e-6;

  private final OptionalDouble motor1TravelRotations;
  private final OptionalDouble motor2TravelRotations;
  private final Source source;

  private ElevatorCalibration(
      OptionalDouble motor1TravelRotations,
      OptionalDouble motor2TravelRotations,
      Source source) {
    this.motor1TravelRotations = motor1TravelRotations;
    this.motor2TravelRotations = motor2TravelRotations;
    this.source = source;
  }

  /** Reads the calibration out of {@link Constants.Elevator}. */
  public static ElevatorCalibration fromConstants() {
    return of(
        Constants.Elevator.MAX_EXTENSION_ROTATIONS,
        Constants.Elevator.MOTOR_1_MAX_EXTENSION_ROTATIONS,
        Constants.Elevator.MOTOR_2_MAX_EXTENSION_ROTATIONS);
  }

  /** One travel serving both motors; the style a symmetric rig fills in. */
  public static ElevatorCalibration ofShared(double travelRotations) {
    return of(travelRotations, Double.NaN, Double.NaN);
  }

  /** A travel per motor; the style a geared rig needs. */
  public static ElevatorCalibration ofPerMotor(
      double motor1TravelRotations, double motor2TravelRotations) {
    return of(Double.NaN, motor1TravelRotations, motor2TravelRotations);
  }

  /**
   * Resolves the shared and per-motor entries. Per-motor values win whenever both are usable; a
   * half-filled pair falls back to the shared value rather than mixing two coordinate systems.
   */
  public static ElevatorCalibration of(double shared, double motor1, double motor2) {
    OptionalDouble motor1Travel = usableTravel(motor1);
    OptionalDouble motor2Travel = usableTravel(motor2);
    if (motor1Travel.isPresent() && motor2Travel.isPresent()) {
      return new ElevatorCalibration(
          motor1Travel, motor2Travel, Source.PER_MOTOR); // per-motor 分电机标定
    }
    OptionalDouble sharedTravel = usableTravel(shared);
    if (sharedTravel.isPresent()) {
      return new ElevatorCalibration(sharedTravel, sharedTravel, Source.SHARED);
    }
    return new ElevatorCalibration(
        OptionalDouble.empty(), OptionalDouble.empty(), Source.UNCALIBRATED);
  }

  private static OptionalDouble usableTravel(double travelRotations) {
    // Magnitudes only: the direction the reading and the voltage move live in their own constants,
    // so a signed travel here would mean two sources of truth about which way is up.
    return Double.isFinite(travelRotations) && travelRotations >= MINIMUM_VALID_TRAVEL_ROTATIONS
        ? OptionalDouble.of(travelRotations)
        : OptionalDouble.empty();
  }

  /** True once usable travel exists for both motors; every movement is refused before that. */
  public boolean isCalibrated() {
    return source != Source.UNCALIBRATED;
  }

  public Source source() {
    return source;
  }

  public double motor1TravelRotations() {
    return motor1TravelRotations.orElseThrow(() -> new IllegalStateException("uncalibrated"));
  }

  public double motor2TravelRotations() {
    return motor2TravelRotations.orElseThrow(() -> new IllegalStateException("uncalibrated"));
  }

  /** Position as a fraction of this motor's own travel; 0 at the zero, 1 at full extension. */
  public double travelFraction(int motor, double extensionRotations) {
    double travel = motor == 1 ? motor1TravelRotations() : motor2TravelRotations();
    return extensionRotations / travel;
  }

  /** Ratio between the two motors' configured travels; 1.0 for a symmetric rig. */
  public double configuredTravelRatio() {
    if (!isCalibrated()) {
      return 0.0;
    }
    return Math.abs(motor2TravelRotations()) / Math.abs(motor1TravelRotations());
  }

  /**
   * True when one shared number is standing in for both motors. Only that style can be wrong
   * unnoticed: the travels themselves say 1:1, so detecting an actual gearing difference has to
   * come from comparing the live encoder readings (see the measured-ratio telemetry the subsystem
   * publishes).
   */
  public boolean reliesOnSharedTravelAssumption() {
    return source == Source.SHARED;
  }
}
