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
    public static final double TRANSLATION_EXPO = 0.5; // Exponential shaping factor for translation joystick input (0 = no shaping, 1 = full shaping)
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

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 19;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 18;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 12;
    public static final double LAUNCHING_FEEDER_VOLTAGE = -12;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = -12;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;

    public static final double DEFAULT_AT_SPEED_RPM = 4500.0;
    public static final double DEFAULT_AT_SPEED_TOL_RPM = 150.0;
    public static final double DEFAULT_AT_SPEED_DEBOUNCE_S = 0.10;
  }
}
