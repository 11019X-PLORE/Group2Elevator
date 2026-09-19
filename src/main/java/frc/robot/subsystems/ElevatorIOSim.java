// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * A mock mechanism behind the same {@link ElevatorIO} contract as the real drives, so the whole
 * control loop — limits, ramp, speed ceiling, presets, the direction fault — can be exercised with
 * no roboRIO attached: on the laptop in desktop simulation, and headless in unit tests.
 *
 * <p><b>This is a stand-in, not an identified model.</b> It integrates one stage position from the
 * applied voltage with a first-order response and a gravity load, and reports each motor's raw
 * reading through {@link Constants.Elevator#EXTENSION_ROTOR_SIGN} so positions travel through the
 * code on exactly the sign path the hardware uses. The two rates below were chosen to feel like a
 * bench elevator at 2 V; nothing here should be mistaken for a measurement of the real mechanism,
 * and a controller tuned against it needs re-validating on the rig.
 */
public class ElevatorIOSim implements ElevatorIO {
  /** Fraction of full travel covered per second per volt of applied movement voltage. */
  private static final double TRAVEL_PER_SECOND_PER_VOLT = 0.6;

  /**
   * Voltage the stage needs before it moves at all: below this the mock gravity load wins and the
   * position drifts back down, which is what makes the gravity feedforward term testable.
   */
  private static final double GRAVITY_BREAKAWAY_VOLTS = 0.2;

  private final double motor1RotationsAtFullTravel;
  private final double motor2RotationsAtFullTravel;
  private final DoubleSupplier clockSeconds;

  private double extensionFraction;
  private double motor1LastAppliedVolts;
  private double motor2LastAppliedVolts;
  private double lastUpdateSeconds;

  /** Desktop-simulation constructor: the real clock and a symmetric mock stage. */
  public ElevatorIOSim() {
    this(1.0, 1.0, Timer::getFPGATimestamp);
  }

  /**
   * @param motor1RotationsAtFullTravel rotor rotations motor 1 covers over the full stroke
   * @param motor2RotationsAtFullTravel the same for motor 2; pass a different value to reproduce a
   *     geared rig like the one ArmDemo measured at 3.4 and 14.5 rotations
   */
  public ElevatorIOSim(double motor1RotationsAtFullTravel, double motor2RotationsAtFullTravel) {
    this(motor1RotationsAtFullTravel, motor2RotationsAtFullTravel, Timer::getFPGATimestamp);
  }

  /** Full seam: a clock a test can step, plus per-motor stage scales. */
  public ElevatorIOSim(
      double motor1RotationsAtFullTravel,
      double motor2RotationsAtFullTravel,
      DoubleSupplier clockSeconds) {
    this.motor1RotationsAtFullTravel = motor1RotationsAtFullTravel;
    this.motor2RotationsAtFullTravel = motor2RotationsAtFullTravel;
    this.clockSeconds = clockSeconds;
    this.lastUpdateSeconds = clockSeconds.getAsDouble();
  }

  @Override
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    integrate();
    inputs.motor1Rotations =
        extensionFraction * motor1RotationsAtFullTravel * Constants.Elevator.EXTENSION_ROTOR_SIGN;
    inputs.motor2Rotations =
        extensionFraction * motor2RotationsAtFullTravel * Constants.Elevator.EXTENSION_ROTOR_SIGN;
    inputs.motor1AppliedVolts = motor1LastAppliedVolts;
    inputs.motor2AppliedVolts = motor2LastAppliedVolts;
    // A mock is always answering, instantly: age 0 and responding true keep the freshness policy
    // satisfied here, and the tests that care about that policy drive the fake IO directly.
    inputs.motor1PositionIsResponding = true;
    inputs.motor2PositionIsResponding = true;
    inputs.motor1PositionAgeMilliseconds = 0.0;
    inputs.motor2PositionAgeMilliseconds = 0.0;
  }

  private void integrate() {
    double nowSeconds = clockSeconds.getAsDouble();
    double dtSeconds = Math.max(0.0, nowSeconds - lastUpdateSeconds);
    lastUpdateSeconds = nowSeconds;
    if (dtSeconds == 0.0) {
      return;
    }
    // The stage follows motor 1's voltage: a mirrored pair drives one mechanism.
    double netVolts = motor1LastAppliedVolts - GRAVITY_BREAKAWAY_VOLTS;
    double deltaFraction = TRAVEL_PER_SECOND_PER_VOLT * netVolts * dtSeconds;
    // The physical stops are hard: no commanded voltage walks the mock stage past either end, so
    // an over-run in the controller shows up as a stalled position rather than a phantom travel.
    extensionFraction = MathUtil.clamp(extensionFraction + deltaFraction, 0.0, 1.0);
  }

  /** Where the mock stage currently is, for assertions in tests. */
  public double getExtensionFraction() {
    return extensionFraction;
  }

  @Override
  public void setMotorVoltages(double motor1Voltage, double motor2Voltage) {
    motor1LastAppliedVolts = motor1Voltage;
    motor2LastAppliedVolts = motor2Voltage;
  }

  @Override
  public void zeroEncoders() {
    extensionFraction = 0.0;
  }
}
