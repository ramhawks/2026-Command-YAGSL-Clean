// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for controlling the shooter mechanism.
 */
public class ShooterSubsystem extends SubsystemBase {
  
  //private final Spark m_shooterMotor;
  private final SparkMax m_shooterMotorSparkMax;
  
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    //m_shooterMotor = new Spark(Constants.ShooterConstants.SHOOTER_MOTOR_PORT);
    m_shooterMotorSparkMax = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_PORT, MotorType.kBrushless);
  }

  public void setSpeed(double speed) {
    m_shooterMotorSparkMax.set(speed);
  }
  
  /**
   * Shoots fuel.
   * 
   * @return a command that shoots fuel
   */
  public Command shootCommand() {
    return run(() -> m_shooterMotorSparkMax.set(Constants.ShooterConstants.SHOOT_SPEED));
  }
  
  /**
   * Stops the shooter.
   * 
   * @return a command that stops the shooter
   */
  public Command stopShooterCommand() {
    return runOnce(() -> m_shooterMotorSparkMax.set(0.0))
        .andThen(() -> m_shooterMotorSparkMax.set(0.0));
  }
  
  /**
   * Stops the shooter.
   */
  public void stop() {
    m_shooterMotorSparkMax.set(0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
