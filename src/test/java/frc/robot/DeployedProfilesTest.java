// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * The CI gate on the deploy directory: every profile file the robot could load must survive the
 * strict loader and stay inside the calibrated travel at every resampled instant, so a corrupt or
 * stale file fails the build instead of a match. This is what makes "commit the config and the
 * JSON together" a checkable discipline rather than a habit.
 */
class DeployedProfilesTest {
  private static final Path PROFILE_DIRECTORY = Path.of("src/main/deploy/profiles");

  @Test
  void everyDeployedProfileFileLoadsAndStaysInsideTheTravel() throws IOException {
    List<Path> files;
    try (Stream<Path> entries = Files.list(PROFILE_DIRECTORY)) {
      files = entries.filter(path -> path.toString().endsWith(".json")).sorted().toList();
    }
    assertFalse(files.isEmpty(), "no profile files are deployed");

    for (Path file : files) {
      MotionProfile profile = MotionProfile.fromJson(file);
      assertFalse(profile.name().isEmpty(), file + " carries no name");
      // The loader checks the stored samples; resampling densely also covers the interpolation
      // between them, which is what the follower actually consumes.
      for (double t = 0.0; t <= profile.durationSeconds(); t += 0.01) {
        MotionProfile.Sample sample = profile.sample(t);
        assertTrue(
            sample.position() >= -1e-9 && sample.position() <= 1.0 + 1e-9,
            file + " leaves the travel at t = " + t + ": " + sample.position());
      }
    }
  }

  @Test
  void theDemoBindingProfileExistsAndStartsFromRestAtTheZero() throws IOException {
    Path demo = Path.of("src/main/deploy").resolve(Constants.Elevator.DEMO_PROFILE_FILE);
    assertTrue(Files.exists(demo), "the demo binding points at " + demo + ", which is missing");

    MotionProfile profile = MotionProfile.fromJson(demo);
    assertEquals(0.0, profile.initialPosition(), 1e-9);
    assertEquals(0.0, profile.sample(0.0).velocity(), 1e-9);
    assertEquals(0.0, profile.sample(profile.durationSeconds()).velocity(), 1e-9);
  }
}
