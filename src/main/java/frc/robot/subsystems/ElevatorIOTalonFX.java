// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** ElevatorIO implementation for the real TalonFX hardware (CAN IDs from {@link Constants.Elevator}). */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX motor1 = new TalonFX(Constants.Elevator.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Elevator.MOTOR_2_ID);
  private final StatusSignal<Angle> motor1Position = motor1.getPosition();
  private final StatusSignal<Angle> motor2Position = motor2.getPosition();
  private double motor1LastAppliedVolts;
  private double motor2LastAppliedVolts;

  public ElevatorIOTalonFX() {
    configureMotor(motor1);
    configureMotor(motor2);
  }

  private static void configureMotor(TalonFX motor) {
    MotorOutputConfigs config =
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
    // One CAN transaction refreshes every signal the subsystem consumes this cycle.
    BaseStatusSignal.refreshAll(motor1Position, motor2Position);
    inputs.motor1Rotations = motor1Position.getValueAsDouble();
    inputs.motor2Rotations = motor2Position.getValueAsDouble();
    inputs.motor1AppliedVolts = motor1LastAppliedVolts;
    inputs.motor2AppliedVolts = motor2LastAppliedVolts;
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
