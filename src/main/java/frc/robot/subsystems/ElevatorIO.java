// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the dual-motor elevator. Sensor values are polled into {@link
 * ElevatorIOInputs} exactly once per cycle by {@link #updateInputs}; commands flow back through
 * the setters. Implementations hide everything vendor specific.
 */
public interface ElevatorIO {
  /**
   * Reads every elevator sensor once and fills {@code inputs}. Implementations should batch
   * their hardware reads into a single transaction so one cycle sees one consistent snapshot.
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
    public double motor1Rotations;
    public double motor2Rotations;
    /** Voltages of the most recent setMotorVoltages() call, reported back for the log. */
    public double motor1AppliedVolts;
    public double motor2AppliedVolts;
  }
}
