// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorIOTalonFX;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (just the scheduler calls). Instead, the structure of the robot (including
 * subsystems, OI devices, and commands) should be declared here. Its job is wiring only: which
 * IO implementation backs the subsystem and which buttons run which commands. The subsystem owns
 * its NetworkTables tuning entries and polls them in its own periodic loop.
 */
public class RobotContainer {
  // The single fill-in limit from Constants is the whole calibration: once it holds the
  // measured travel, the elevator is ready to run.
  private final ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem(new ElevatorIOTalonFX(), Constants.Elevator.MAX_EXTENSION_ROTATIONS);

  private final CommandPS5Controller driverController =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains the subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Deliberately no startup zeroEncoders(): the TalonFX positions must survive a robot-code
    // restart, or a mid-match soft restart would re-zero wherever the mechanism happens to be
    // and shift every software limit. After a full power cycle the encoders read zero on their
    // own, so the elevator must be at its physical zero position before power-on (see the
    // README safety warning).
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
  }
}
