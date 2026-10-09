// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The profile loader and sampler, on their own: what a valid file yields, what a corrupt one must
 * refuse, and how sampling interpolates and clamps. The follower that consumes these samples is
 * exercised in ElevatorSubsystemTest.
 */
class MotionProfileTest {
  private static final double TOL = 1e-9;

  private static final String VALID_JSON =
      """
      {
        "metadata": { "name": "testProfile", "notes": "placeholder limits" },
        "samples": [
          { "t": 0.0, "x": 0.25, "v": 0.0, "a": 0.0 },
          { "t": 1.0, "x": 0.50, "v": 0.5, "a": 0.0 },
          { "t": 2.0, "x": 0.75, "v": 0.0, "a": 0.0 }
        ]
      }
      """;

  @Test
  void loadsSamplesAndMetadataFromJson() {
    MotionProfile profile = MotionProfile.fromJson(VALID_JSON);

    assertEquals("testProfile", profile.name());
    assertEquals("placeholder limits", profile.notes());
    assertEquals(2.0, profile.durationSeconds(), TOL);
    assertEquals(0.25, profile.initialPosition(), TOL);
    assertEquals(0.75, profile.finalPosition(), TOL);
  }

  @Test
  void sampleInterpolatesLinearlyBetweenSamples() {
    MotionProfile profile = MotionProfile.fromJson(VALID_JSON);

    MotionProfile.Sample sample = profile.sample(0.5);

    assertEquals(0.5, sample.timeSeconds(), TOL);
    assertEquals(0.375, sample.position(), TOL);
    assertEquals(0.25, sample.velocity(), TOL);
    assertEquals(0.0, sample.acceleration(), TOL);
  }

  @Test
  void sampleClampsToTheRestingEndsOfTheTrajectory() {
    MotionProfile profile = MotionProfile.fromJson(VALID_JSON);

    assertEquals(0.25, profile.sample(-1.0).position(), TOL);
    assertEquals(0.75, profile.sample(99.0).position(), TOL);
    assertEquals(0.0, profile.sample(99.0).velocity(), TOL);
    assertEquals(0.0, profile.sample(99.0).acceleration(), TOL);
  }

  @Test
  void solverNoiseInsideTheEpsilonIsClampedIntoTheTravel() {
    MotionProfile profile =
        MotionProfile.fromJson(
            """
            {
              "samples": [
                { "t": 0.0, "x": -1e-9, "v": 0.0, "a": 0.0 },
                { "t": 1.0, "x": 1.0000000005, "v": 0.0, "a": 0.0 }
              ]
            }
            """);

    assertEquals(0.0, profile.initialPosition(), TOL);
    assertEquals(1.0, profile.finalPosition(), TOL);
  }

  @Test
  void rejectsATrajectoryThatLeavesTheCalibratedTravel() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotionProfile.fromJson(
                """
                {
                  "samples": [
                    { "t": 0.0, "x": 0.0, "v": 0.0, "a": 0.0 },
                    { "t": 1.0, "x": 1.2, "v": 0.0, "a": 0.0 }
                  ]
                }
                """));
  }

  @Test
  void rejectsATrajectoryThatDoesNotStartAtTimeZero() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotionProfile.fromJson(
                """
                {
                  "samples": [
                    { "t": 0.5, "x": 0.0, "v": 0.0, "a": 0.0 },
                    { "t": 1.0, "x": 0.5, "v": 0.0, "a": 0.0 }
                  ]
                }
                """));
  }

  @Test
  void rejectsNonIncreasingTime() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotionProfile.fromJson(
                """
                {
                  "samples": [
                    { "t": 0.0, "x": 0.0, "v": 0.0, "a": 0.0 },
                    { "t": 1.0, "x": 0.5, "v": 0.0, "a": 0.0 },
                    { "t": 1.0, "x": 0.6, "v": 0.0, "a": 0.0 }
                  ]
                }
                """));
  }

  @Test
  void rejectsMissingFieldsNonFiniteValuesAndOneSampleTrajectories() {
    assertThrows(
        IllegalArgumentException.class,
        () -> MotionProfile.fromJson("{\"samples\": [{\"t\": 0.0, \"x\": 0.0}]}"));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotionProfile.fromJson(
                """
                {
                  "samples": [
                    { "t": 0.0, "x": 0.0, "v": 0.0, "a": 0.0 }
                  ]
                }
                """));
    assertThrows(
        IllegalArgumentException.class,
        () -> MotionProfile.fromJson("{\"samples\": []}"));
    assertThrows(IllegalArgumentException.class, () -> MotionProfile.fromJson("not json"));
    assertTrue(assertThrows(
            IllegalArgumentException.class,
            () ->
                MotionProfile.of(
                    "bad",
                    new double[] {0.0, 1.0},
                    new double[] {0.0, Double.NaN},
                    new double[] {0.0, 0.0},
                    new double[] {0.0, 0.0}))
        .getMessage()
        .contains("bad"));
  }
}
