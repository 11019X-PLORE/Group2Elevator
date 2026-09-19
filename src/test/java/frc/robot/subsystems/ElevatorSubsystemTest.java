// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.ElevatorCalibration;
import org.junit.jupiter.api.Test;

/**
 * Subsystem behavior, with no hardware and no wall clock: a fake IO supplies positions and reports
 * what was asked for, and a manual clock steps the 20 ms loop so ramps, settle windows and arrival
 * windows are asserted rather than slept through.
 */
class ElevatorSubsystemTest {
  private static final double TOL = 1e-9;
  private static final double CYCLE_SECONDS = 0.02;
  /** ArmDemo's measured bench travels: the two motors do not cover the same number of rotations. */
  private static final double MOTOR_1_TRAVEL = 3.4;
  private static final double MOTOR_2_TRAVEL = 14.5;

  // ---------------------------------------------------------------------------
  // Position reads and calibration

  @Test
  void reportsBothEncoderPositionsAsPositiveWhileExtending() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));

    assertEquals(1.0, elevator.getMotor1ExtensionRotations(), TOL);
    assertEquals(2.0, elevator.getMotor2ExtensionRotations(), TOL);
    assertEquals(1.5, elevator.getExtensionRotations(), TOL);
  }

  @Test
  void refusesEveryMovementUntilTheLimitIsFilledIn() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(0.0));

    assertFalse(elevator.isCalibrated());
    assertFalse(elevator.canDrive());

    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();
    elevator.periodic();
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals(0.0, io.motor2Voltage, TOL);
  }

  @Test
  void treatsASignedTravelAsUnfilledBecauseDirectionHasItsOwnConstants() {
    // The previous version of this project asked for a negative travel to mean "wound the other
    // way". Signs now live in EXTENSION_ROTOR_SIGN / EXTENSION_VOLTAGE_SIGN, so a signed travel is
    // ambiguous input and must refuse to move rather than guess which way it was meant.
    assertFalse(ElevatorCalibration.ofShared(-3.4).isCalibrated());
    FakeElevatorIO io = new FakeElevatorIO();
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(-3.4));
    assertFalse(elevator.isCalibrated());
  }

  @Test
  void perMotorTravelsWinAndAPartialPairFallsBackToTheSharedValue() {
    ElevatorCalibration perMotor = ElevatorCalibration.ofPerMotor(MOTOR_1_TRAVEL, MOTOR_2_TRAVEL);
    assertEquals(ElevatorCalibration.Source.PER_MOTOR, perMotor.source());
    assertEquals(MOTOR_1_TRAVEL, perMotor.motor1TravelRotations(), TOL);
    assertEquals(MOTOR_2_TRAVEL, perMotor.motor2TravelRotations(), TOL);

    ElevatorCalibration halfFilled =
        ElevatorCalibration.of(8.95, MOTOR_1_TRAVEL, Double.NaN);
    assertEquals(ElevatorCalibration.Source.SHARED, halfFilled.source());
    assertEquals(8.95, halfFilled.motor1TravelRotations(), TOL);
    assertEquals(8.95, halfFilled.motor2TravelRotations(), TOL);
  }

  @Test
  void reachesTheUpperLimitWhenEitherMotorReachesItsOwnTravel() {
    FakeElevatorIO motor1AtLimitIO = new FakeElevatorIO();
    motor1AtLimitIO.motor1Rotations = -MOTOR_1_TRAVEL;
    motor1AtLimitIO.motor2Rotations = -2.0;
    ElevatorSubsystem elevator1 = newRig(motor1AtLimitIO, gearedCalibration());

    FakeElevatorIO motor2AtLimitIO = new FakeElevatorIO();
    motor2AtLimitIO.motor1Rotations = -2.0;
    motor2AtLimitIO.motor2Rotations = -MOTOR_2_TRAVEL;
    ElevatorSubsystem elevator2 = newRig(motor2AtLimitIO, gearedCalibration());

    assertTrue(elevator1.atUpperLimit());
    assertTrue(elevator2.atUpperLimit());
  }

  @Test
  void gearedRigStillReachesItsFullPhysicalTravelBeforeStopping() {
    // The regression this project's calibration model used to fail: normalizing both motors by one
    // averaged travel tripped "whichever arrives first" at 8.95 of motor 2's 14.5 rotations, so the
    // elevator stopped at about 62% of its stroke and could never reach the top.
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -MOTOR_1_TRAVEL;
    io.motor2Rotations = -MOTOR_2_TRAVEL;
    ElevatorSubsystem elevator = newRig(io, gearedCalibration());

    elevator.periodic();
    assertTrue(elevator.atUpperLimit());
    assertEquals(1.0, elevator.getExtensionFraction(), 1e-6);
  }

  @Test
  void fractionStaysBelowOneUntilBothMotorsCoverTheirOwnTravel() {
    FakeElevatorIO io = new FakeElevatorIO();
    // Motor 2 three quarters of the way up, motor 1 only halfway.
    io.motor1Rotations = -MOTOR_1_TRAVEL * 0.5;
    io.motor2Rotations = -MOTOR_2_TRAVEL * 0.75;
    ElevatorSubsystem elevator = newRig(io, gearedCalibration());

    elevator.periodic();
    assertEquals(0.625, elevator.getExtensionFraction(), 1e-6);
    assertFalse(elevator.atUpperLimit());
  }

  @Test
  void midPresetSitsAtTheSamePhysicalHeightForBothMotors() {
    FakeElevatorIO halfwayUp = new FakeElevatorIO();
    halfwayUp.motor1Rotations = -MOTOR_1_TRAVEL * 0.5;
    halfwayUp.motor2Rotations = -MOTOR_2_TRAVEL * 0.5;
    ElevatorSubsystem elevator = newRig(halfwayUp, gearedCalibration());
    elevator.periodic();

    assertTrue(elevator.atMidExtension());
    assertTrue(elevator.atOrBelowMidExtension());
  }

  @Test
  void flagsASharedTravelThatIsHidingTwoUnequalMotors() {
    FakeElevatorIO io = new FakeElevatorIO();
    // One shared limit was filled in, but the raw readings are four times apart: that rig needs
    // per-motor travels and the subsystem now says so instead of quietly shortening the stroke.
    io.motor1Rotations = -0.8;
    io.motor2Rotations = -3.2;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.2));

    elevator.periodic();

    elevator.periodic();

    assertTrue(elevator.isCalibrated());
    assertTrue(elevator.calibrationIsSuspect(), "one shared travel is hiding a geared rig");
  }

  // ---------------------------------------------------------------------------
  // Outputs and limit commands

  @Test
  void appliesOppositeTwoVoltOutputsForExtensionAndRetraction() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));

    runMode(elevator, elevator.moveToUpperLimitCommand(), 1);
    assertEquals(2.0, io.motor1Voltage, TOL);
    assertEquals(-2.0, io.motor2Voltage, TOL);

    runMode(elevator, elevator.moveToLowerLimitCommand(), 1);
    assertEquals(-2.0, io.motor1Voltage, TOL);
    assertEquals(2.0, io.motor2Voltage, TOL);
  }

  @Test
  void extensionCommandStopsBothMotorsWhenEitherUpperLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToUpperLimitCommand();

    runMode(elevator, command, 1);
    assertFalse(command.isFinished());

    io.motor1Rotations = -3.4;
    elevator.periodic();
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals(0.0, io.motor2Voltage, TOL);
    command.end(false);
  }

  @Test
  void retractionCommandStopsBothMotorsWhenEitherLowerLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToLowerLimitCommand();

    runMode(elevator, command, 1);
    assertFalse(command.isFinished());

    io.motor2Rotations = 0.0;
    elevator.periodic();
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals(0.0, io.motor2Voltage, TOL);
    command.end(false);
  }

  @Test
  void theSubsystemCutsTheOutputInTheSameCycleItSeesTheLimit() {
    // The old design tested its limits inside execute(), which CommandScheduler runs before the
    // subsystem's periodic(), so every check acted on a position from 20 ms ago and the last
    // approved voltage stayed on the wires for a cycle past the stop. Here the freshest snapshot of
    // the cycle is polled and applied in one place, so no separate command bookkeeping is needed.
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -3.3;
    io.motor2Rotations = -3.3;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();
    elevator.periodic();
    assertEquals(2.0, io.motor1Voltage, TOL);

    io.motor1Rotations = -3.4;
    elevator.periodic();

    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals(0.0, io.motor2Voltage, TOL);
    command.end(false);
  }

  @Test
  void midCommandExtendsToTheMiddleExtensionAndStops() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -1.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToMidExtensionCommand();

    runMode(elevator, command, 1);
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, TOL);

    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atMidExtension());
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals("ElevatorToMidExtension", elevator.getLastMoveActionName());
    assertTrue(elevator.getLastMoveDurationSeconds() >= 0.0);
  }

  @Test
  void midCommandRetractsToTheMiddleExtensionFromAbove() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -3.4;
    io.motor2Rotations = -3.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToMidExtensionCommand();

    runMode(elevator, command, 1);
    assertEquals(-2.0, io.motor1Voltage, TOL);

    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atOrBelowMidExtension());
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, TOL);
  }

  @Test
  void midCommandFinishesImmediatelyWhenOneMotorIsAlreadyAtItsMid() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 0.0; // motor 1 fully retracted
    io.motor2Rotations = -1.7; // motor 2 exactly at the shared half travel
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToMidExtensionCommand();

    runMode(elevator, command, 1);

    // The average (0.25) latches extension, and motor 2 already satisfies the "whichever mechanism
    // arrives first" rule, so the command ends without driving.
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, TOL);
  }

  @Test
  void movementCommandsRecordTheirDurationAndName() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ManualClock clock = new ManualClock();
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io, ElevatorCalibration.ofShared(3.4), ElevatorSubsystem.Tunables.ceilingsOff(), clock);
    Command command = elevator.moveToUpperLimitCommand();

    runMode(elevator, command, 5, clock);
    command.end(false);

    assertEquals("ElevatorToUpperLimit", elevator.getLastMoveActionName());
    assertEquals(0.1, elevator.getLastMoveDurationSeconds(), 1e-6);
  }

  // ---------------------------------------------------------------------------
  // Trust in the sensors

  @Test
  void stopsDrivingWhenADriveStopsAnswering() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));
    Command command = elevator.moveToUpperLimitCommand();
    runMode(elevator, command, 1);
    assertEquals(2.0, io.motor1Voltage, TOL);

    io.motor1PositionIsResponding = false;
    elevator.periodic();

    assertFalse(elevator.sensorsAreTrusted());
    assertEquals(0.0, io.motor1Voltage, TOL);
    command.end(false);
  }

  @Test
  void stopsDrivingWhenPositionsGoStaleAndMovesAgainWhenTheyDoNot() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    io.motor1PositionAgeMilliseconds = 500.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));

    runMode(elevator, elevator.moveToUpperLimitCommand(), 1);
    assertEquals(0.0, io.motor1Voltage, TOL);

    io.motor1PositionAgeMilliseconds = 5.0;
    elevator.periodic();
    assertEquals(2.0, io.motor1Voltage, TOL);
  }

  @Test
  void settingTheTimeoutToZeroDisablesOnlyTheAgeCheck() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    io.motor1PositionAgeMilliseconds = 500.0;
    // Tunables ceiling-off leaves the age check itself at its default budget; this rig pins the
    // supplier to 0, which means "no age limit" rather than "no sensors".
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(3.4),
            ageLimitOff(),
            () -> 0.0);

    runMode(elevator, elevator.moveToUpperLimitCommand(), 1);
    assertEquals(2.0, io.motor1Voltage, TOL);

    io.motor1PositionIsResponding = false;
    elevator.periodic();
    assertEquals(0.0, io.motor1Voltage, TOL);
  }

  // ---------------------------------------------------------------------------
  // Direction fault

  @Test
  void latchesAFaultWhenTheMechanismMovesAgainstItsCommand() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.7;
    io.motor2Rotations = -1.7;
    ManualClock clock = new ManualClock();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, ElevatorCalibration.ofShared(3.4), ElevatorSubsystem.Tunables.ceilingsOff(), clock);
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();

    for (int cycle = 0; cycle < 12 && !elevator.directionIsReversed(); cycle++) {
      // The rig is being driven "up" but the readings fall: a wrong sign constant, or the two CAN
      // IDs swapped. Walk the raw positions positive, which the correction turns into retraction.
      io.motor1Rotations += 0.1;
      io.motor2Rotations += 0.1;
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
    }

    assertTrue(elevator.directionIsReversed());
    assertFalse(elevator.canDrive());
    clock.advance(CYCLE_SECONDS);
    elevator.periodic();
    assertEquals(0.0, io.motor1Voltage, TOL);
    command.end(false);
  }

  @Test
  void doesNotLatchTheFaultJustBecauseTheMechanismIsNotMoving() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.7;
    io.motor2Rotations = -1.7;
    ManualClock clock = new ManualClock();
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, ElevatorCalibration.ofShared(3.4), ElevatorSubsystem.Tunables.ceilingsOff(), clock);
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();

    for (int cycle = 0; cycle < 20; cycle++) {
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
    }

    assertFalse(elevator.directionIsReversed());
    assertEquals(2.0, io.motor1Voltage, TOL);
    command.end(false);
  }

  // ---------------------------------------------------------------------------
  // Motion ceilings

  @Test
  void slewRampsTheVoltageAtMostTheConfiguredRate() {
    assertEquals(1.2, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 60.0), TOL);
    assertEquals(0.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.0, 60.0), TOL);
    assertEquals(2.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 0.0), TOL);
  }

  @Test
  void speedCapCutsTheVoltageOnlyInTheTooFastDirection() {
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.5, 1.5), TOL);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.49, 1.5), TOL);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, -10.0, 1.5), TOL);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 10.0, 0.0), TOL);
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.5, 1.5), TOL);
    assertEquals(-2.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.49, 1.5), TOL);
  }

  @Test
  void nonFiniteTunableValuesFallBackToTheValidatedDefaults() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    // A mistyped dashboard entry must not disable the ramp: with the 60 V/s default restored and a
    // manual clock that has not advanced, dt is 0 and the output stays at zero volts.
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io, ElevatorCalibration.ofShared(3.4), nanTunables(), () -> 0.0);

    runMode(elevator, elevator.moveToUpperLimitCommand(), 1);

    assertEquals(0.0, io.motor1Voltage, TOL);
    assertEquals(0.0, io.motor2Voltage, TOL);
  }

  @Test
  void directionReversalsRampThroughZeroInsteadOfSnapping() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -2.0;
    double[] slewRate = {1.0e7};
    ManualClock clock = new ManualClock();
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io, ElevatorCalibration.ofShared(3.4), ElevatorSubsystem.Tunables.withSlew(() -> slewRate[0]), clock);

    // Ramp disabled: build up to the full +2V in one cycle, no sleeping required.
    runMode(elevator, elevator.moveToUpperLimitCommand(), 1, clock);
    assertEquals(2.0, io.motor1Voltage, 0.1);

    // Then reverse at 6 V/s. The tunable is re-read at the top of each cycle, so the first
    // ramped retract moves the voltage only 6 * 0.02 = 0.12V from +2V, not to -2V.
    slewRate[0] = 6.0;
    runMode(elevator, elevator.moveToLowerLimitCommand(), 1, clock);

    assertEquals(1.88, io.motor1Voltage, TOL);
    assertEquals(-io.motor1Voltage, io.motor2Voltage, TOL);
  }

  // ---------------------------------------------------------------------------
  // Position presets

  @Test
  void presetDrivesTowardItsTargetAndEndsInsideTheArriveWindow() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 0.0; // at the bottom
    io.motor2Rotations = 0.0;
    ManualClock clock = new ManualClock();
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(4.0),
            ElevatorSubsystem.Tunables.withPositionGains(2.0, 0.02),
            clock);
    Command command = elevator.moveToFractionCommand(0.5);
    command.initialize();

    int cycles = 0;
    while (!command.isFinished() && cycles < 200) {
      // The mechanism climbs in step with what the controller asks for.
      io.motor1Rotations -= 0.02 * 4.0 * 0.25;
      io.motor2Rotations = io.motor1Rotations;
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
      cycles++;
    }
    command.end(false);

    assertTrue(command.isFinished(), "preset never arrived");
    assertEquals(0.5, elevator.getExtensionFraction(), 0.05);
    assertTrue(elevator.getLastMoveDurationSeconds() > 0.0);
  }

  @Test
  void presetRefusesToPushPastAHardLimit() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -4.0; // already at the top
    io.motor2Rotations = -4.0;
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(4.0),
            ElevatorSubsystem.Tunables.withPositionGains(2.0, 0.02),
            () -> 0.0);
    Command command = elevator.moveToFractionCommand(1.5); // clamped to 1.0
    command.initialize();
    elevator.periodic();

    assertEquals(1.0, elevator.getTargetExtensionFraction(), TOL);
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, TOL);
    command.end(false);
  }

  @Test
  void holdCommandKeepsItsRequestAfterArriving() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -2.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(4.0),
            ElevatorSubsystem.Tunables.withPositionGains(2.0, 0.02),
            () -> 0.0);
    Command command = elevator.holdAtFractionCommand(0.5);
    command.initialize();
    elevator.periodic();

    assertFalse(command.isFinished(), "a hold must not finish on its own");
    assertEquals(0.0, io.motor1Voltage, TOL); // arrived, and the feedforward is still 0 V
    command.end(true);
    assertEquals(ElevatorSubsystem.MotionMode.STOPPED, elevator.getMode());
  }

  @Test
  void gravityFeedforwardIsWhatSitsAtTheTarget() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -2.0;
    io.motor2Rotations = -2.0;
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(4.0),
            new ElevatorSubsystem.Tunables(
                () -> 0.0, () -> 0.0, () -> 0.0, () -> 2.0, () -> 0.02, () -> 0.6),
            () -> 0.0);
    Command command = elevator.holdAtFractionCommand(0.5);
    command.initialize();
    elevator.periodic();

    assertEquals(0.6, io.motor1Voltage, TOL);
    assertEquals(-0.6, io.motor2Voltage, TOL);
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, TOL);
  }

  // ---------------------------------------------------------------------------
  // Re-zeroing

  @Test
  void rezeroIsAcceptedOnlyOnTheLowerStop() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -1.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));

    assertFalse(elevator.rezeroAtLowerLimit());
    assertFalse(io.requestedZero, "zeroing mid-travel would move every software limit");

    io.motor1Rotations = 0.0;
    io.motor2Rotations = 0.0;
    elevator.periodic(); // the guard reads the freshest polled snapshot
    assertTrue(elevator.rezeroAtLowerLimit());
    assertTrue(io.requestedZero);
    assertEquals(0.0, elevator.getMotor1ExtensionRotations(), TOL);
  }

  @Test
  void rezeroCommandRunsTheSameGuard() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 0.0;
    io.motor2Rotations = 0.0;
    ElevatorSubsystem elevator = newRig(io, ElevatorCalibration.ofShared(3.4));

    Command command = elevator.rezeroAtLowerLimitCommand();
    command.initialize();
    command.execute();

    assertTrue(io.requestedZero);
    assertTrue(command.isFinished());
  }

  // ---------------------------------------------------------------------------
  // End to end against the mock mechanism

  @Test
  void simulatedElevatorRunsFromTheBottomToItsFullTravelAndStops() {
    ManualClock clock = new ManualClock();
    ElevatorIOSim io = new ElevatorIOSim(3.4, 14.5, clock);
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io, ElevatorCalibration.ofPerMotor(3.4, 14.5), ElevatorSubsystem.Tunables.ceilingsOff(), clock);
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();

    int cycles = 0;
    while (!command.isFinished() && cycles < 400) {
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
      cycles++;
    }
    command.end(false);

    assertTrue(command.isFinished(), "the simulated elevator never reached its limit");
    assertEquals(1.0, io.getExtensionFraction(), 0.05);
    assertTrue(cycles < 400, "took too long: " + cycles);
  }

  @Test
  void simulatedElevatorStopsShortWhenCalibratedOnOneAveragedTravel() {
    // What the previous calibration model did, kept as a characterisation test so it cannot come
    // back: one averaged number on a geared rig leaves the elevator parked well below the top,
    // with nothing in the log to say so.
    ManualClock clock = new ManualClock();
    ElevatorIOSim io = new ElevatorIOSim(3.4, 14.5, clock);
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared((3.4 + 14.5) / 2.0),
            ElevatorSubsystem.Tunables.ceilingsOff(),
            clock);
    Command command = elevator.moveToUpperLimitCommand();
    command.initialize();

    int cycles = 0;
    while (!command.isFinished() && cycles < 400) {
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
      cycles++;
    }
    command.end(false);

    assertTrue(command.isFinished());
    // The stage is nowhere near the top even though the software says it hit the limit.
    assertTrue(io.getExtensionFraction() < 0.7, "expected a short stop, got " + io.getExtensionFraction());
    assertTrue(elevator.atUpperLimit());
  }

  @Test
  void simulatedPresetWithoutFeedforwardParksShortOfItsTarget() {
    ManualClock clock = new ManualClock();
    ElevatorIOSim io = new ElevatorIOSim(3.4, 3.4, clock);
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(3.4),
            ElevatorSubsystem.Tunables.withPositionGains(4.0, 0.02),
            clock);
    Command command = elevator.holdAtFractionCommand(0.6);
    command.initialize();

    for (int cycle = 0; cycle < 300; cycle++) {
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
    }

    command.end(false);
    // kP 4 V/travel against the mock's 0.2 V breakaway: the proportional term cannot produce any
    // torque inside 0.05 of travel, so a feedforward-free loop parks about that far below target.
    assertEquals(0.55, io.getExtensionFraction(), 0.02);
  }

  @Test
  void simulatedPresetWithGravityFeedforwardHoldsItsTarget() {
    ManualClock clock = new ManualClock();
    ElevatorIOSim io = new ElevatorIOSim(3.4, 3.4, clock);
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(
            io,
            ElevatorCalibration.ofShared(3.4),
            new ElevatorSubsystem.Tunables(
                () -> 0.0, () -> 0.0, () -> 0.0, () -> 4.0, () -> 0.02, () -> 0.2),
            clock);
    Command command = elevator.holdAtFractionCommand(0.6);
    command.initialize();

    for (int cycle = 0; cycle < 300; cycle++) {
      clock.advance(CYCLE_SECONDS);
      elevator.periodic();
    }
    command.end(false);

    assertEquals(0.6, io.getExtensionFraction(), 0.02);
  }

  // ---------------------------------------------------------------------------
  // Helpers

  private static ElevatorCalibration gearedCalibration() {
    return ElevatorCalibration.ofPerMotor(MOTOR_1_TRAVEL, MOTOR_2_TRAVEL);
  }

  /** A rig on a frozen clock with the ceilings at their defaults: dt is always 0. */
  private static ElevatorSubsystem newRig(FakeElevatorIO io, ElevatorCalibration calibration) {
    return new ElevatorSubsystem(
        io, calibration, ElevatorSubsystem.Tunables.ceilingsOff(), () -> 0.0);
  }

  private static ElevatorSubsystem.Tunables ageLimitOff() {
    return new ElevatorSubsystem.Tunables(
        () -> 0.0, () -> 0.0, () -> 0.0, () -> 2.0, () -> 0.02, () -> 0.0);
  }

  private static ElevatorSubsystem.Tunables nanTunables() {
    return new ElevatorSubsystem.Tunables(
        () -> Double.NaN, () -> Double.NaN, () -> Double.NaN, () -> 2.0, () -> 0.02, () -> 0.0);
  }

  private static void runMode(ElevatorSubsystem elevator, Command command, int cycles) {
    command.initialize();
    for (int cycle = 0; cycle < cycles; cycle++) {
      command.execute();
      elevator.periodic();
    }
  }

  /** Same, but stepping the clock one loop period before each cycle. */
  private static void runMode(
      ElevatorSubsystem elevator, Command command, int cycles, ManualClock clock) {
    command.initialize();
    for (int cycle = 0; cycle < cycles; cycle++) {
      clock.advance(CYCLE_SECONDS);
      command.execute();
      elevator.periodic();
    }
  }

  private static final class ManualClock implements DoubleSupplier {
    private double seconds;

    @Override
    public double getAsDouble() {
      return seconds;
    }

    void advance(double deltaSeconds) {
      seconds += deltaSeconds;
    }
  }

  /** Records what the subsystem asked for and lets a test set what the drives report. */
  private static final class FakeElevatorIO implements ElevatorIO {
    double motor1Rotations;
    double motor2Rotations;
    double motor1Voltage;
    double motor2Voltage;
    double motor1PositionAgeMilliseconds;
    double motor2PositionAgeMilliseconds;
    boolean motor1PositionIsResponding = true;
    boolean motor2PositionIsResponding = true;
    boolean requestedZero;

    @Override
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
      inputs.motor1Rotations = motor1Rotations;
      inputs.motor2Rotations = motor2Rotations;
      inputs.motor1AppliedVolts = motor1Voltage;
      inputs.motor2AppliedVolts = motor2Voltage;
      inputs.motor1PositionIsResponding = motor1PositionIsResponding;
      inputs.motor2PositionIsResponding = motor2PositionIsResponding;
      inputs.motor1PositionAgeMilliseconds = motor1PositionAgeMilliseconds;
      inputs.motor2PositionAgeMilliseconds = motor2PositionAgeMilliseconds;
    }

    @Override
    public void zeroEncoders() {
      requestedZero = true;
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
