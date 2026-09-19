// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** How the calibration constants resolve into the coordinate every limit check uses. */
class ElevatorCalibrationTest {
  @Test
  void nothingFilledInReportsUncalibrated() {
    ElevatorCalibration calibration = ElevatorCalibration.of(0.0, Double.NaN, Double.NaN);

    assertFalse(calibration.isCalibrated());
    assertEquals(ElevatorCalibration.Source.UNCALIBRATED, calibration.source());
  }

  @Test
  void aNonFiniteOrNegativeTravelIsTreatedAsUnfilled() {
    assertFalse(ElevatorCalibration.ofShared(Double.NaN).isCalibrated());
    assertFalse(ElevatorCalibration.ofShared(Double.POSITIVE_INFINITY).isCalibrated());
    // Direction has its own constants, so a signed travel is ambiguous input, not a second style.
    assertFalse(ElevatorCalibration.ofShared(-8.95).isCalibrated());
    assertFalse(ElevatorCalibration.ofShared(1e-9).isCalibrated());
  }

  @Test
  void aSharedTravelServesBothMotors() {
    ElevatorCalibration calibration = ElevatorCalibration.ofShared(8.95);

    assertEquals(ElevatorCalibration.Source.SHARED, calibration.source());
    assertEquals(8.95, calibration.motor1TravelRotations(), 1e-9);
    assertEquals(8.95, calibration.motor2TravelRotations(), 1e-9);
    assertEquals(1.0, calibration.configuredTravelRatio(), 1e-9);
    assertTrue(calibration.reliesOnSharedTravelAssumption());
  }

  @Test
  void bothMotorsReachOneAtTheSamePhysicalHeightWhenTravelsDiffer() {
    // The case the shared value gets wrong: on ArmDemo's bench elevator motor 2 covers 14.5
    // rotations while motor 1 covers 3.4, so full extension is 3.4 on one encoder and 14.5 on the
    // other at the same instant.
    ElevatorCalibration calibration = ElevatorCalibration.ofPerMotor(3.4, 14.5);

    assertFalse(calibration.reliesOnSharedTravelAssumption());
    assertEquals(1.0, calibration.travelFraction(1, 3.4), 1e-9);
    assertEquals(1.0, calibration.travelFraction(2, 14.5), 1e-9);
    assertEquals(0.5, calibration.travelFraction(1, 1.7), 1e-9);
    assertEquals(0.5, calibration.travelFraction(2, 7.25), 1e-9);
    assertEquals(14.5 / 3.4, calibration.configuredTravelRatio(), 1e-9);
  }

  @Test
  void aHalfFilledPairFallsBackToTheSharedValueInsteadOfMixingCoordinates() {
    ElevatorCalibration calibration = ElevatorCalibration.of(8.95, 3.4, 0.0);

    assertEquals(ElevatorCalibration.Source.SHARED, calibration.source());
    assertEquals(8.95, calibration.motor2TravelRotations(), 1e-9);
  }

  @Test
  void constantsDriveTheResolution() {
    ElevatorCalibration fromRepoDefaults = ElevatorCalibration.fromConstants();

    // As shipped, nothing is measured yet, so the elevator must refuse to move.
    assertFalse(fromRepoDefaults.isCalibrated());
    assertEquals(0.0, fromRepoDefaults.configuredTravelRatio(), 1e-9);
  }
}
