// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import java.io.IOException;
import java.nio.file.Path;

/**
 * An immutable point-to-point motion profile in travel-fraction space, produced offline by
 * {@code tools/profile-generator} and loaded from the deploy directory.
 *
 * <p>A profile is a sampled position/velocity/acceleration trajectory: minimum-time under
 * velocity/acceleration/jerk limits, at rest at both ends, positions strictly inside the
 * calibrated travel. The subsystem samples it once per cycle and turns the sample into volts;
 * everything safety-relevant stays in the subsystem — the runtime ceilings (slew, speed cap,
 * hard limits) still bound whatever the follower computes, so a profile can only ever shape a
 * move, not authorize one.
 *
 * <p>Loading is deliberately strict, in the same spirit as the calibration constants: a profile
 * that cannot prove it starts at t = 0, moves forward in time, and stays inside the travel is
 * rejected at load time — before it can be followed — rather than discovered mid-move. The one
 * tolerance is a 1e-6 epsilon for solver noise (the generator writes exact zeros, but a
 * hand-edited file deserves the same forgiveness the arithmetic deserves) and positions inside
 * that epsilon are clamped into the travel.
 */
public final class MotionProfile {
  /** One sampled state of the trajectory, in travel-fraction units. */
  public record Sample(double timeSeconds, double position, double velocity, double acceleration) {}

  /** Position tolerance at load: beyond this, an out-of-travel sample rejects the file. */
  private static final double POSITION_EPSILON = 1.0e-6;

  /** Time tolerance at load: the first sample must start at t = 0 within this. */
  private static final double TIME_EPSILON = 1.0e-6;

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private final String name;
  private final String notes;
  private final double[] times;
  private final double[] positions;
  private final double[] velocities;
  private final double[] accelerations;

  private MotionProfile(
      String name, String notes, double[] times, double[] positions, double[] velocities,
      double[] accelerations) {
    this.name = name;
    this.notes = notes;
    this.times = times;
    this.positions = positions;
    this.velocities = velocities;
    this.accelerations = accelerations;
  }

  /**
   * Programmatic construction (tests and callers that build a profile in code). Runs the same
   * validation as the JSON loader.
   */
  public static MotionProfile of(
      String name,
      double[] times,
      double[] positions,
      double[] velocities,
      double[] accelerations) {
    validate(name, times, positions, velocities, accelerations);
    return new MotionProfile(
        name, "", times.clone(), clampPositions(positions), velocities.clone(),
        accelerations.clone());
  }

  /** Loads and validates a profile file; the profile name defaults to the file's stem. */
  public static MotionProfile fromJson(Path path) throws IOException {
    JsonNode root = MAPPER.readTree(path.toFile());
    if (root == null || !root.has("samples") || !root.get("samples").isArray()) {
      throw new IOException("profile file has no \"samples\" array: " + path);
    }
    JsonNode sampleNodes = root.get("samples");
    double[] times = new double[sampleNodes.size()];
    double[] positions = new double[sampleNodes.size()];
    double[] velocities = new double[sampleNodes.size()];
    double[] accelerations = new double[sampleNodes.size()];
    for (int i = 0; i < sampleNodes.size(); i++) {
      JsonNode node = sampleNodes.get(i);
      times[i] = requireFinite(node, "t", path);
      positions[i] = requireFinite(node, "x", path);
      velocities[i] = requireFinite(node, "v", path);
      accelerations[i] = requireFinite(node, "a", path);
    }
    String defaultName = path.getFileName().toString().replaceFirst("\\.json$", "");
    JsonNode metadata = root.get("metadata");
    String name = textOrDefault(metadata, "name", defaultName);
    String notes = textOrDefault(metadata, "notes", "");
    validate(name, times, positions, velocities, accelerations);
    return new MotionProfile(
        name, notes, times.clone(), clampPositions(positions), velocities.clone(),
        accelerations.clone());
  }

  /** Same loader for an in-memory document, so tests need no files. */
  public static MotionProfile fromJson(String json) {
    try {
      JsonNode root = MAPPER.readTree(json);
      if (root == null || !root.has("samples") || !root.get("samples").isArray()) {
        throw new IllegalArgumentException("profile document has no \"samples\" array");
      }
      JsonNode sampleNodes = root.get("samples");
      double[] times = new double[sampleNodes.size()];
      double[] positions = new double[sampleNodes.size()];
      double[] velocities = new double[sampleNodes.size()];
      double[] accelerations = new double[sampleNodes.size()];
      for (int i = 0; i < sampleNodes.size(); i++) {
        JsonNode node = sampleNodes.get(i);
        times[i] = requireFinite(node, "t", null);
        positions[i] = requireFinite(node, "x", null);
        velocities[i] = requireFinite(node, "v", null);
        accelerations[i] = requireFinite(node, "a", null);
      }
      JsonNode metadata = root.get("metadata");
      String name = textOrDefault(metadata, "name", "profile");
      String notes = textOrDefault(metadata, "notes", "");
      validate(name, times, positions, velocities, accelerations);
      return new MotionProfile(
          name, notes, times.clone(), clampPositions(positions), velocities.clone(),
          accelerations.clone());
    } catch (IOException exception) {
      throw new IllegalArgumentException("profile document is not valid JSON", exception);
    }
  }

  private static double requireFinite(JsonNode node, String field, Path path) {
    JsonNode value = node.get(field);
    if (value == null || !value.isNumber() || !Double.isFinite(value.asDouble())) {
      throw new IllegalArgumentException(
          "sample is missing a finite \"" + field + "\"" + (path == null ? "" : ": " + path));
    }
    return value.asDouble();
  }

  private static String textOrDefault(JsonNode metadata, String field, String fallback) {
    return metadata != null && metadata.has(field) ? metadata.get(field).asText() : fallback;
  }

  private static void validate(
      String name,
      double[] times,
      double[] positions,
      double[] velocities,
      double[] accelerations) {
    int length = times.length;
    if (length != positions.length
        || length != velocities.length
        || length != accelerations.length) {
      throw new IllegalArgumentException("profile " + name + ": sample arrays have different lengths");
    }
    if (length < 2) {
      throw new IllegalArgumentException(
          "profile " + name + ": needs at least a first and last sample");
    }
    if (Math.abs(times[0]) > TIME_EPSILON) {
      throw new IllegalArgumentException(
          "profile " + name + ": must start at t = 0, starts at " + times[0]);
    }
    double[][] arrays = {times, positions, velocities, accelerations};
    for (double[] array : arrays) {
      for (double entry : array) {
        if (!Double.isFinite(entry)) {
          throw new IllegalArgumentException(
              "profile " + name + ": contains a non-finite sample value");
        }
      }
    }
    for (int i = 1; i < length; i++) {
      if (times[i] <= times[i - 1]) {
        throw new IllegalArgumentException(
            "profile " + name + ": time must increase, sample " + i + " does not");
      }
    }
    for (int i = 0; i < length; i++) {
      if (positions[i] < -POSITION_EPSILON || positions[i] > 1.0 + POSITION_EPSILON) {
        throw new IllegalArgumentException(
            "profile " + name + ": sample " + i + " leaves the calibrated travel ("
                + positions[i] + ")");
      }
    }
  }

  private static double[] clampPositions(double[] positions) {
    double[] clamped = new double[positions.length];
    for (int i = 0; i < positions.length; i++) {
      clamped[i] = MathUtil.clamp(positions[i], 0.0, 1.0);
    }
    return clamped;
  }

  public String name() {
    return name;
  }

  /** The generator's note carried in the file's metadata (e.g. a PLACEHOLDER-limits warning). */
  public String notes() {
    return notes;
  }

  public double durationSeconds() {
    return times[times.length - 1];
  }

  /** Where the mechanism must be when the profile starts; the start waypoint. */
  public double initialPosition() {
    return positions[0];
  }

  public double finalPosition() {
    return positions[positions.length - 1];
  }

  /**
   * The sampled state at a time, clamped to the trajectory's ends: before the start returns the
   * first sample, past the end the last (which the generator guarantees is at rest at the goal,
   * so holding past the end is holding the destination). Between samples the four states are
   * interpolated linearly, which at the generator's 200 intervals is indistinguishable from the
   * solved curve at loop rate.
   */
  public Sample sample(double timeSeconds) {
    if (timeSeconds <= times[0]) {
      return sampleAt(0, times[0]);
    }
    if (timeSeconds >= durationSeconds()) {
      return sampleAt(times.length - 1, times[times.length - 1]);
    }
    int index = highestIndexAtOrBefore(timeSeconds);
    double span = times[index + 1] - times[index];
    double alpha = (timeSeconds - times[index]) / span;
    return new Sample(
        timeSeconds,
        lerp(positions[index], positions[index + 1], alpha),
        lerp(velocities[index], velocities[index + 1], alpha),
        lerp(accelerations[index], accelerations[index + 1], alpha));
  }

  private Sample sampleAt(int index, double timeSeconds) {
    return new Sample(timeSeconds, positions[index], velocities[index], accelerations[index]);
  }

  /** Largest sample index whose time is at or before the given time; times are sorted. */
  private int highestIndexAtOrBefore(double timeSeconds) {
    int low = 0;
    int high = times.length - 1;
    while (high - low > 1) {
      int middle = (low + high) / 2;
      if (times[middle] <= timeSeconds) {
        low = middle;
      } else {
        high = middle;
      }
    }
    return low;
  }

  private static double lerp(double from, double to, double alpha) {
    return from + alpha * (to - from);
  }
}
