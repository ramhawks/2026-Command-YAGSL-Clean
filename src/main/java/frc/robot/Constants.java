// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final String SWERVE_DIR = "deploy/swerve/neo";
  
  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 0;
    public static final double LEFT_Y_DEADBAND = 0.25;
    public static final double LEFT_X_DEADBAND = 0.25;
  }

  public static class DriverConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_PORT = 19;  // Adjust port as needed
    public static final double INTAKE_SPEED = 0.8;  // 0-1.0
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_PORT = 18;  // Adjust port as needed
    public static final double SHOOT_SPEED = 0.8;    // 0-1.0
    public static final double SPIN_UP_SECONDS = 1.0; // Time to spin up shooter before launching
  }
}
