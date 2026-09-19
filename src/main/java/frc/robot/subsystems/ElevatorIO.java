// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the dual-motor elevator. Sensor values are polled into {@link
 * ElevatorIOInputs} exactly once per cycle by {@link #updateInputs}; commands flow back through the
 * setters. Implementations hide everything vendor specific.
 */
public interface ElevatorIO {
  /**
   * Reads every elevator sensor once and fills {@code inputs}. Implementations should batch their
   * hardware reads into a single transaction so one cycle sees one consistent snapshot, and should
   * say whether that snapshot is trustworthy rather than handing back frozen values as if they were
   * fresh.
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Applies mirrored voltages: motor 1 receives {@code motor1Voltage}, motor 2 always receives
   * its negation (the two mechanisms wind in opposite directions).
   */
  default void setMotorVoltages(double motor1Voltage, double motor2Voltage) {}

  /** Defines the current position as the calibration zero. */
  default void zeroEncoders() {}

  /** Everything read from the elevator hardware in one cycle; logged wholesale for replay. */
  @AutoLog
  class ElevatorIOInputs {
    /**
     * Rotor positions in rotations, already latency-compensated where the hardware supports it, in
     * the drive's raw sign convention (this mechanism winds negative while extending).
     */
    public double motor1Rotations;
    public double motor2Rotations;
    /** Voltages of the most recent setMotorVoltages() call, reported back for the log. */
    public double motor1AppliedVolts;
    public double motor2AppliedVolts;
    /**
     * False when this drive has never answered or its most recent status transaction failed. Paired
     * with the ages below, the subsystem decides when a reading has gone stale enough to stop on —
     * a frozen position that still reads "not at the limit" is what grinds a mechanism into its
     * stops, so "do not trust the limits" has to be a first-class input, not an afterthought.
     */
    public boolean motor1PositionIsResponding;
    public boolean motor2PositionIsResponding;
    /** Age of the newest position read in milliseconds; +inf when the drive never reported. */
    public double motor1PositionAgeMilliseconds;
    public double motor2PositionAgeMilliseconds;
  }
}
