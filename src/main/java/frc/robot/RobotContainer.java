// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorIO;
import frc.robot.subsystems.ElevatorIOTalonFX;
import frc.robot.subsystems.ElevatorIOSim;
import frc.robot.subsystems.ElevatorSubsystem;
import java.io.IOException;

import java.util.Optional;

/**
 * The wiring: which IO implementation backs the subsystem, which calibration it runs against, and
 * which buttons run which commands. It holds no tuning data and no control logic — the subsystem
 * owns its NetworkTables entries and every movement decision, and {@link Robot} owns which hardware
 * layer is live.
 */
public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem;

  private final CommandPS5Controller driverController =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * @param useRealHardware false on the laptop, where {@link ElevatorIOSim} stands in for the two
   *     drives so the mechanism can be exercised without a roboRIO
   */
  public RobotContainer(boolean useRealHardware) {
    ElevatorIO io = useRealHardware ? new ElevatorIOTalonFX() : new ElevatorIOSim();
    // The calibration constants are the whole setup: pass them in here, at the point of assembly,
    // rather than letting the subsystem read them behind everyone's back.
    elevatorSubsystem = new ElevatorSubsystem(io, ElevatorCalibration.fromConstants());
    // Deliberately no startup zeroEncoders(): the TalonFX positions must survive a robot-code
    // restart, or a mid-match soft restart would re-zero wherever the mechanism happens to be and
    // shift every software limit. After a full power cycle the encoders read zero on their own, so
    // the elevator must be at its physical zero position before power-on (see the README warning).
    configureBindings();
  }

  private void configureBindings() {
    // This controller reports Xbox-style raw button IDs in Driver Station. Each binding uses
    // onTrue, so one press starts the command and it runs until the software limit is reached.
    driverController
        .button(OperatorConstants.ELEVATOR_EXTEND_BUTTON)
        .onTrue(elevatorSubsystem.moveToUpperLimitCommand());
    driverController
        .button(OperatorConstants.ELEVATOR_RETRACT_BUTTON)
        .onTrue(elevatorSubsystem.moveToLowerLimitCommand());
    driverController
        .button(OperatorConstants.ELEVATOR_MID_BUTTON)
        .onTrue(elevatorSubsystem.moveToMidExtensionCommand());
    // Two new bindings, and the raw IDs are the ones worth verifying on your own pad before
    // relying on them: both commands are simply unreachable if a button never reports.
    driverController
        .button(OperatorConstants.ELEVATOR_REZERO_BUTTON)
        .onTrue(elevatorSubsystem.rezeroAtLowerLimitCommand());
    driverController
        .button(OperatorConstants.ELEVATOR_WORK_PRESET_BUTTON)
        .onTrue(
            elevatorSubsystem.moveToFractionCommand(Constants.Elevator.WORK_EXTENSION_FRACTION));
    // Demo of the profile follower: one file from the deploy directory, one button. The profiles
    // shipped today carry PLACEHOLDER limits (see their metadata) — regenerate them from
    // tools/profile-generator after the bench characterization before trusting their timing.
    loadProfile(Constants.Elevator.DEMO_PROFILE_FILE)
        .ifPresent(
            profile ->
                driverController
                    .button(OperatorConstants.ELEVATOR_PROFILE_BUTTON)
                    .onTrue(elevatorSubsystem.followProfileCommand(profile)));
    // Hold-to-run: the button applies the dashboard characterization voltage, releasing it stops.
    driverController
        .button(OperatorConstants.ELEVATOR_CHARACTERIZE_BUTTON)
        .whileTrue(elevatorSubsystem.characterizationVoltageCommand());
  }

  /**
   * Loads a profile relative to the deploy directory (which desktop simulation also serves). A
   * missing or malformed file logs and leaves the binding absent rather than failing robot
   * startup — an unloadable profile must cost one button, not the whole program.
   */
  private static Optional<MotionProfile> loadProfile(String relativePath) {
    try {
      return Optional.of(
          MotionProfile.fromJson(
              Filesystem.getDeployDirectory().toPath().resolve(relativePath)));
    } catch (IOException | RuntimeException exception) {
      System.err.printf(
          "Elevator profile %s could not be loaded: %s%n", relativePath, exception.getMessage());
      return Optional.empty();
    }
  }
}
