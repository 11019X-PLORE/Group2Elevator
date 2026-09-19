// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * ElevatorIO implementation for the real TalonFX hardware (CAN IDs from {@link
 * Constants.Elevator}).
 *
 * <p>Two things happen here beyond simply reading the encoders, because this mechanism's only
 * position protection is computed in Java:
 *
 * <ul>
 *   <li><b>Latency compensation.</b> A status signal carries the position as of whenever the drive
 *       last sent it, so that read is already milliseconds stale before the 20 ms loop adds its
 *       own lag. Positions are therefore extrapolated to "now" with the paired velocity signal,
 *       which is what {@code BaseStatusSignal.getLatencyCompensatedValueAsDouble} exists for.
 *   <li><b>Freshness reporting.</b> A dropped CAN transaction leaves the cached value sitting
 *       there, and a stale position that still reads "not at the limit" is precisely what keeps
 *       driving a mechanism into its stops. Each cycle reports whether the drive has ever answered,
 *       whether the last transaction was clean, and how old the reading is. Deciding how old is too
 *       old is the subsystem's job, so the threshold stays a single live tunable.
 * </ul>
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  private static final double NEVER_UPDATED_AGE_MILLISECONDS = Double.POSITIVE_INFINITY;

  private final TalonFX motor1 = new TalonFX(Constants.Elevator.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Elevator.MOTOR_2_ID);
  private final StatusSignal<Angle> motor1Position = motor1.getRotorPosition();
  private final StatusSignal<Angle> motor2Position = motor2.getRotorPosition();
  private final StatusSignal<AngularVelocity> motor1Velocity = motor1.getRotorVelocity();
  private final StatusSignal<AngularVelocity> motor2Velocity = motor2.getRotorVelocity();
  private double motor1LastAppliedVolts;
  private double motor2LastAppliedVolts;

  public ElevatorIOTalonFX() {
    configureMotor(motor1);
    configureMotor(motor2);
    // Ask for the signals at a fixed rate instead of inheriting defaults: 100 Hz gives the 20 ms
    // loop five samples' worth of margin, at a CAN load a two-motor bus absorbs easily.
    reportFailure(
        "signal rates",
        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.Elevator.STATUS_SIGNAL_UPDATE_FREQUENCY_HZ,
            motor1Position,
            motor1Velocity,
            motor2Position,
            motor2Velocity));
  }

  private static void configureMotor(TalonFX motor) {
    MotorOutputConfigs config = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    reportFailure("neutral mode", motor.getConfigurator().apply(config));
  }

  private static void reportFailure(String what, StatusCode code) {
    if (code == null || code.isOK()) {
      return;
    }
    System.err.printf("Elevator TalonFX %s configuration failed: %s%n", what, code.getName());
  }

  @Override
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    // One CAN transaction refreshes every signal the subsystem consumes this cycle.
    BaseStatusSignal.refreshAll(motor1Position, motor1Velocity, motor2Position, motor2Velocity);
    // Extrapolate each position to "now" along its own velocity instead of acting on the instant
    // the drive happened to last report.
    inputs.motor1Rotations =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(motor1Position, motor1Velocity);
    inputs.motor2Rotations =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(motor2Position, motor2Velocity);
    inputs.motor1PositionAgeMilliseconds = signalAgeMilliseconds(motor1Position);
    inputs.motor2PositionAgeMilliseconds = signalAgeMilliseconds(motor2Position);
    inputs.motor1PositionIsResponding =
        BaseStatusSignal.isAllGood(motor1Position, motor1Velocity);
    inputs.motor2PositionIsResponding =
        BaseStatusSignal.isAllGood(motor2Position, motor2Velocity);
    inputs.motor1AppliedVolts = motor1LastAppliedVolts;
    inputs.motor2AppliedVolts = motor2LastAppliedVolts;
  }

  /** Milliseconds since this signal last arrived, or +inf if it never has. */
  private static double signalAgeMilliseconds(StatusSignal<Angle> signal) {
    if (!signal.hasUpdated()) {
      return NEVER_UPDATED_AGE_MILLISECONDS;
    }
    return Timer.getFPGATimestamp() * 1.0e3 - signal.getTimestamp().getTime() / 1.0e3;
  }

  @Override
  public void setMotorVoltages(double motor1Voltage, double motor2Voltage) {
    motor1LastAppliedVolts = motor1Voltage;
    motor2LastAppliedVolts = motor2Voltage;
    motor1.setVoltage(motor1Voltage);
    motor2.setVoltage(motor2Voltage);
  }

  @Override
  public void zeroEncoders() {
    motor1.setPosition(0.0);
    motor2.setPosition(0.0);
  }
}
