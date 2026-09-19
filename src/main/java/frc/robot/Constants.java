// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide numerical constants. This class holds values only: the calibration arithmetic lives in
 * {@link ElevatorCalibration}, and everything an operator tunes mid-match is a NetworkTables entry
 * owned by {@link frc.robot.subsystems.ElevatorSubsystem}.
 */
public final class Constants {
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    // Same raw button numbers the elevator used in ArmDemo (Xbox-style IDs in Driver Station).
    public static final int ELEVATOR_EXTEND_BUTTON = 3;
    public static final int ELEVATOR_RETRACT_BUTTON = 1;
    public static final int ELEVATOR_MID_BUTTON = 6;
    // New in this project, and deliberately on the two shoulder buttons so nothing collides with
    // the ArmDemo-proven bindings above. Verify the raw IDs on your own controller in Driver
    // Station before relying on them: both commands are inert if the button never reports.
    public static final int ELEVATOR_REZERO_BUTTON = 7;
    public static final int ELEVATOR_WORK_PRESET_BUTTON = 8;

    private OperatorConstants() {}
  }

  public static final class Elevator {
    public static final int MOTOR_1_ID = 11;
    public static final int MOTOR_2_ID = 12;

    // =========================================================================
    // THE CALIBRATION (标定). Fill in motor rotations from the calibrated zero
    // (elevator fully down) to full extension, direction-corrected so that
    // extension is positive. See docs/CALIBRATION.md for the measurement steps
    // and for why the average of two unequal travels is the one wrong number.
    //
    // Two supported styles:
    //   * Symmetric rig (both motors travel the same amount): fill
    //     MAX_EXTENSION_ROTATIONS and leave both per-motor values NaN.
    //   * Geared rig (ArmDemo measured 3.4 vs 14.5 rotations on its bench
    //     elevator): fill MOTOR_1_... and MOTOR_2_... per motor. The shared
    //     value is then ignored, and the subsystem reports the mismatch.
    //
    // Travels are positive magnitudes. Any non-finite or non-positive entry counts as "not filled
    // in": while no travel is configured the subsystem reports uncalibrated and every movement
    // refuses to run, so an unfilled or mistyped constant can never drive the mechanism.
    // =========================================================================
    public static final double MAX_EXTENSION_ROTATIONS = 0.0;
    public static final double MOTOR_1_MAX_EXTENSION_ROTATIONS = Double.NaN;
    public static final double MOTOR_2_MAX_EXTENSION_ROTATIONS = Double.NaN;

    // Two independent physical facts, kept apart on purpose rather than folded into one sign that
    // has to stand for both. EXTENSION_ROTOR_SIGN says what the raw encoder reading does while the
    // elevator extends (-1: it falls, which is what this bench rig was observed doing);
    // EXTENSION_VOLTAGE_SIGN says which motor-1 polarity extends it. Both come out of the same
    // hand-raise test in docs/CALIBRATION.md, and if either is filled in backwards the mechanism
    // demonstrably moves against its command, which the subsystem latches as a fault instead of
    // driving away from its limits.
    public static final int EXTENSION_ROTOR_SIGN = -1;
    public static final int EXTENSION_VOLTAGE_SIGN = 1;

    // A rig whose motors travel 5% apart is still fine to drive on one shared limit; beyond that
    // the shared value silently shortens the reachable travel, so surface it in telemetry.
    public static final double TRAVEL_ASYMMETRY_WARNING_RATIO = 1.05;

    // Fixed movement voltage magnitude (validated smooth on this mechanism). Motor 1
    // receives its sign; motor 2 always receives the negation (the two wind opposite).
    public static final double MOVEMENT_VOLTAGE = 2.0;

    // Button 6 moves the elevator to the middle working extension from whichever side it
    // is on, expressed as a fraction of each motor's own calibrated travel.
    public static final double MID_EXTENSION_FRACTION = 0.5;

    // Button 8 runs the position controller to this fraction of travel instead of
    // open-looping into a limit. Every preset has to stay inside the calibrated travel.
    public static final double WORK_EXTENSION_FRACTION = 0.75;

    // Spec ceilings, same as ArmDemo: speed — each motor's extension is normalized by its
    // own travel, and when the faster motor's fraction-per-second reaches the cap while
    // moving, both movement voltages are cut to zero. Acceleration — movement voltages ramp
    // at most this many volts per second. Live-tunable on NetworkTables; 0 or a non-finite
    // value disables that ceiling (a mistyped dashboard entry falls back to these defaults).
    public static final double DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND = 1.5;
    public static final double DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND = 60.0;

    // Position-controller defaults. NOT bench-validated: they are the values this project
    // starts from, exposed on NetworkTables so the team can retune without reflashing. The
    // gravity feedforward deliberately starts at zero volts rather than a guessed mass.
    public static final double DEFAULT_POSITION_KP_VOLTS_PER_TRAVEL = 2.0;
    public static final double DEFAULT_POSITION_ARRIVE_TOLERANCE_FRACTION = 0.02;
    public static final double DEFAULT_GRAVITY_FEEDFORWARD_VOLTS = 0.0;

    // TalonFX sensor handling.
    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY_HZ = 100.0;
    // How long a position signal may go un-refreshed before it is treated as untrustworthy.
    // Live-tunable, and 0 disables the age check outright (the "device never reported" and
    // "transaction failed" checks stay active either way).
    public static final double DEFAULT_SENSOR_TIMEOUT_MILLISECONDS = 100.0;

    private Elevator() {}
  }

  private Constants() {}
}
