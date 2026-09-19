// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  private static final double MAX_EXTENSION_ROTATIONS = 3.4;

  @Test
  void reportsBothEncoderPositionsAsPositiveWhileExtending() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    assertEquals(1.0, elevator.getMotor1ExtensionRotations(), 1e-9);
    assertEquals(2.0, elevator.getMotor2ExtensionRotations(), 1e-9);
    assertEquals(1.5, elevator.getExtensionRotations(), 1e-9);
  }

  @Test
  void refusesEveryMovementUntilTheLimitIsFilledIn() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, 0.0, () -> 0.0, () -> 0.0);

    assertFalse(elevator.isCalibrated());

    elevator.extend();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    elevator.retract();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    elevator.moveTowardMidExtension();
    assertEquals(0.0, io.motor1Voltage, 1e-9);

    // Movement commands also end immediately instead of driving an uncalibrated elevator.
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();
    command.execute();
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void zeroesBothEncodersAtTheKnownMinimumLength() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 8.0;
    io.motor2Rotations = -8.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    elevator.zeroEncoders();

    assertEquals(0.0, io.motor1Rotations, 1e-9);
    assertEquals(0.0, io.motor2Rotations, 1e-9);
    assertEquals(0.0, elevator.getMotor1ExtensionRotations(), 1e-9);
    assertEquals(0.0, elevator.getMotor2ExtensionRotations(), 1e-9);
  }

  @Test
  void reachesTheUpperLimitWhenEitherMotorReachesTheSharedMaximum() {
    FakeElevatorIO motor1AtLimitIO = new FakeElevatorIO();
    motor1AtLimitIO.motor1Rotations = -3.4;
    motor1AtLimitIO.motor2Rotations = -2.0;
    ElevatorSubsystem elevator1 = new ElevatorSubsystem(motor1AtLimitIO, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    FakeElevatorIO motor2AtLimitIO = new FakeElevatorIO();
    motor2AtLimitIO.motor1Rotations = -2.0;
    motor2AtLimitIO.motor2Rotations = -3.4;
    ElevatorSubsystem elevator2 = new ElevatorSubsystem(motor2AtLimitIO, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    assertTrue(elevator1.atUpperLimit());
    assertTrue(elevator2.atUpperLimit());
  }

  @Test
  void appliesOppositeTwoVoltOutputsForExtensionAndRetraction() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    elevator.extend();
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    elevator.retract();
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void followsTheMeasuredDirectionWhenCalibrationIsNegative() {
    FakeElevatorIO io = new FakeElevatorIO();
    // A rig wound the opposite way: raw positions increase while extending, so the measured
    // limit is negative. Filling it negative must flip every voltage sign automatically.
    io.motor1Rotations = 1.0;
    io.motor2Rotations = 2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, -MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);

    assertTrue(elevator.isCalibrated());
    elevator.extend();
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);

    // Retracting back toward the calibrated zero mirrors the sign too.
    elevator.retract();
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void extensionCommandStopsBothMotorsWhenEitherUpperLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToUpperLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    io.motor1Rotations = -3.4;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void retractionCommandStopsBothMotorsWhenEitherLowerLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToLowerLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);

    io.motor2Rotations = 0.0;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void slewRampsTheVoltageAtMostTheConfiguredRate() {
    assertEquals(1.2, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 60.0), 1e-9);
    assertEquals(0.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.0, 60.0), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 0.0), 1e-9);
  }

  @Test
  void speedCapCutsTheVoltageOnlyInTheTooFastDirection() {
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.5, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.49, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, -10.0, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 10.0, 0.0), 1e-9);
    // Retracting at the cap (negative voltage) is cut the same way.
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.5, 1.5), 1e-9);
    assertEquals(-2.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.49, 1.5), 1e-9);
  }

  @Test
  void nonFiniteSlewValuesFallBackToTheDefaultRamp() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io, MAX_EXTENSION_ROTATIONS, () -> Double.NaN, () -> Double.NaN);

    // A mistyped NetworkTables value must not disable the ramp: it falls back to the 60 V/s
    // default, and called microseconds after construction the ramp has barely moved.
    elevator.extend();
    assertTrue(io.motor1Voltage < 0.5);
    assertEquals(-io.motor1Voltage, io.motor2Voltage, 1e-9);
  }

  @Test
  void directionReversalsRampThroughZeroInsteadOfSnapping() throws InterruptedException {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    double[] slewRate = {1.0e7};
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> slewRate[0]);

    // Guarantee the ramp sees a nonzero dt on the first movement (the desktop clock can
    // report two calls inside the same microsecond, which would read as "no time passed").
    Thread.sleep(2);

    // Build up to the full +2V with the ramp effectively disabled.
    elevator.extend();
    assertEquals(2.0, io.motor1Voltage, 0.1);

    // Then reverse with a slow 6 V/s ramp: tuning values are polled once per cycle, so the
    // periodic() call is when the new rate takes effect — and the next retract is
    // microseconds later, meaning the voltage must still be near +2V (heading for -2V
    // through zero), not snapped to -2V.
    slewRate[0] = 6.0;
    elevator.periodic();
    elevator.retract();
    assertEquals(2.0, io.motor1Voltage, 0.1);
    assertEquals(-io.motor1Voltage, io.motor2Voltage, 1e-9);
  }

  @Test
  void midCommandExtendsToTheMiddleExtensionAndStops() {
    FakeElevatorIO io = new FakeElevatorIO();
    // Both motors start below the middle share (each at ~0.29 of the one shared travel).
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -1.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    // Motor 1 reaching its half-travel share (1.7 rotations) stops both motors.
    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atMidExtension());
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
    assertEquals("ElevatorToMidExtension", elevator.getLastMoveActionName());
    assertTrue(elevator.getLastMoveDurationSeconds() >= 0.0);
  }

  @Test
  void midCommandRetractsToTheMiddleExtensionFromAbove() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -3.4;
    io.motor2Rotations = -3.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);

    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atOrBelowMidExtension());
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void midCommandFinishesImmediatelyWhenOneMotorIsAlreadyPastItsOwnMid() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 0.0; // motor 1 fully retracted
    io.motor2Rotations = -1.7; // motor 2 exactly at the shared half travel
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();

    // The average travel fraction (0.25) latches extension, and motor 2 already satisfies
    // the "whichever mechanism arrives first" rule, so the command stops without moving.
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void movementCommandsRecordTheirDurationAndName() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, MAX_EXTENSION_ROTATIONS, () -> 0.0, () -> 0.0);
    Command command = elevator.moveToUpperLimitCommand();

    command.initialize();
    command.execute();

    io.motor1Rotations = -3.4;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);

    assertEquals("ElevatorToUpperLimit", elevator.getLastMoveActionName());
    assertTrue(elevator.getLastMoveDurationSeconds() >= 0.0);
  }

  private static class FakeElevatorIO implements ElevatorIO {
    double motor1Rotations;
    double motor2Rotations;
    double motor1Voltage;
    double motor2Voltage;

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
      inputs.motor1Rotations = motor1Rotations;
      inputs.motor2Rotations = motor2Rotations;
      inputs.motor1AppliedVolts = motor1Voltage;
      inputs.motor2AppliedVolts = motor2Voltage;
    }

    @Override
    public void zeroEncoders() {
      motor1Rotations = 0.0;
      motor2Rotations = 0.0;
    }

    @Override
    public void setMotorVoltages(double motor1Voltage, double motor2Voltage) {
      this.motor1Voltage = motor1Voltage;
      this.motor2Voltage = motor2Voltage;
    }
  }
}
