// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    // Same raw button numbers the elevator used in ArmDemo (Xbox-style IDs in Driver Station).
    public static final int ELEVATOR_EXTEND_BUTTON = 3;
    public static final int ELEVATOR_RETRACT_BUTTON = 1;
    public static final int ELEVATOR_MID_BUTTON = 6;

    private OperatorConstants() {}
  }

  public static final class Elevator {
    public static final int MOTOR_1_ID = 11;
    public static final int MOTOR_2_ID = 12;

    // =========================================================================
    // THE ONE VALUE TO FILL IN (唯一需要填写的限位).
    //
    // Direction-corrected motor rotations from the calibrated zero (elevator fully
    // down) to full extension, averaged over both motors. Measured by hand-raising
    // the elevator from its physical zero with the robot disabled and reading
    // AdvantageScope's "Elevator/ExtensionRotations" at the top (or the raw TalonFX
    // positions, negated and averaged).
    //
    // The sign is the measured extension direction: a rig wound the opposite way
    // simply fills the negative measurement and every voltage sign follows it.
    //
    // While this stays 0.0 (or holds a mistyped value) the subsystem reports
    // "uncalibrated" and every movement command refuses to run — nothing moves
    // until a real limit is here.
    // =========================================================================
    public static final double MAX_EXTENSION_ROTATIONS = 0.0;

    // Fixed movement voltage magnitude (validated smooth on this mechanism). Motor 1
    // receives its sign; motor 2 always receives the negation (the two wind opposite).
    public static final double MOVEMENT_VOLTAGE = 2.0;

    // Button 6 moves the elevator to the middle working extension from whichever side
    // it is on, expressed as a fraction of the one calibrated travel.
    public static final double MID_EXTENSION_FRACTION = 0.5;

    // Spec ceilings, same as ArmDemo: speed — each motor's extension is normalized by
    // the one shared travel, and when the faster motor's fraction-per-second reaches
    // the cap while moving, both movement voltages are cut to zero. Acceleration —
    // movement voltages ramp at most this many volts per second. Live-tunable on
    // NetworkTables; 0 or non-finite disables.
    public static final double DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND = 1.5;
    public static final double DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND = 60.0;

    private Elevator() {}
  }

  private Constants() {}
}
