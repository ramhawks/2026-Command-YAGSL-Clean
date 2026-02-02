// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for controlling the fuel intake mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {
  
  private final Spark m_intakeMotor;
  
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    m_intakeMotor = new Spark(Constants.IntakeConstants.INTAKE_MOTOR_PORT);
  }
  
  /**
   * Runs the intake motor to collect fuel.
   * 
   * @return a command that runs the intake
   */
  public Command intakeCommand() {
    return run(() -> m_intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED));
  }
  
  /**
   * Reverses the intake motor.
   * 
   * @return a command that reverses the intake
   */
  public Command reverseIntakeCommand() {
    return run(() -> m_intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED));
  }
  
  /**
   * Stops the intake motor.
   * 
   * @return a command that stops the intake
   */
  public Command stopIntakeCommand() {
    return runOnce(() -> m_intakeMotor.set(0.0))
        .andThen(() -> m_intakeMotor.set(0.0));
  }
  
  /**
   * Stops the intake.
   */
  public void stop() {
    m_intakeMotor.set(0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
