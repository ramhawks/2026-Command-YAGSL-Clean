// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;          // shooter
  private final SparkMax intakeLauncherRoller;  // hopper

  // Encoder for the launcher roller.
  private final RelativeEncoder launcherEncoder;

  private final Timer atSpeedTimer = new Timer();

  // ************  NT Tables  ************
  private final NetworkTable tuningFuel = NetworkTableInstance.getDefault().getTable("Tuning").getSubTable("Fuel");
  private final NetworkTable telemetryFuel = NetworkTableInstance.getDefault().getTable("Telemetry").getSubTable("Fuel");

  // ---- Tunable entries ----
  private final NetworkTableEntry intakeFeederVolts         = tuningFuel.getEntry("IntakeFeederVolts");
  private final NetworkTableEntry intakeLauncherVolts       = tuningFuel.getEntry("IntakeLauncherVolts");
  private final NetworkTableEntry launchFeederVolts         = tuningFuel.getEntry("LaunchFeederVolts");
  private final NetworkTableEntry launchLauncherVolts       = tuningFuel.getEntry("LaunchLauncherVolts");
  private final NetworkTableEntry spinupFeederVolts         = tuningFuel.getEntry("SpinupFeederVolts");
  private final NetworkTableEntry launcherAtSpeedRPM        = tuningFuel.getEntry("LauncherAtSpeedRPM");
  private final NetworkTableEntry launcherAtSpeedTolRPM     = tuningFuel.getEntry("LauncherAtSpeedTolRPM");
  private final NetworkTableEntry launcherAtSpeedDebounceS  = tuningFuel.getEntry("LauncherAtSpeedDebounceS");

  // ---- Telemetry entries (for Elastic/AdvantageScope live) ----
  private final NetworkTableEntry feederAppliedVoltsEntry   = telemetryFuel.getEntry("FeederAppliedVolts");
  private final NetworkTableEntry launcherAppliedVoltsEntry = telemetryFuel.getEntry("LauncherAppliedVolts");
  private final NetworkTableEntry launcherRpmEntry          = telemetryFuel.getEntry("LauncherRPM");
  private final NetworkTableEntry atSpeedEntry              = telemetryFuel.getEntry("AtSpeed");


  /** Creates a new CANBallSubsystem. */
  // TODO: Re-evaluate spinUp method. Currently the feeder spins up to speed before launch and the intakeLauncher remains the same speed
  //       during spin-up. Do we want to set the feeder roller to 0 during spin-up and spin up the launcher roller?

  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    launcherEncoder = intakeLauncherRoller.getEncoder();

    // Put the defaults values for various fuel operations into the Network Table.
    // All methods in this subsystem pull values from the Network Table. This allows you to tune the values easily.
    // Replace tuned values in the Constants.java file.
    setDefaultDouble(intakeFeederVolts,        INTAKING_FEEDER_VOLTAGE);
    setDefaultDouble(intakeLauncherVolts,      INTAKING_INTAKE_VOLTAGE);
    setDefaultDouble(launchFeederVolts,        LAUNCHING_FEEDER_VOLTAGE);
    setDefaultDouble(launchLauncherVolts,      LAUNCHING_LAUNCHER_VOLTAGE);
    setDefaultDouble(spinupFeederVolts,        SPIN_UP_FEEDER_VOLTAGE);
    setDefaultDouble(launcherAtSpeedRPM,       DEFAULT_AT_SPEED_RPM);
    setDefaultDouble(launcherAtSpeedTolRPM,    DEFAULT_AT_SPEED_TOL_RPM);
    setDefaultDouble(launcherAtSpeedDebounceS, DEFAULT_AT_SPEED_DEBOUNCE_S);

    // create the configuration for the feeder roller, set a current limit and apply the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(intakeFeederVolts.getDouble(INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(intakeLauncherVolts.getDouble(INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller.setVoltage(-intakeFeederVolts.getDouble(INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(-intakeLauncherVolts.getDouble(INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(launchFeederVolts.getDouble(LAUNCHING_FEEDER_VOLTAGE)); // shooter
    intakeLauncherRoller.setVoltage(launchLauncherVolts.getDouble(LAUNCHING_LAUNCHER_VOLTAGE)); // hopper
  }

  // A method to spin up the launcher roller while spinning the feeder roller to push Fuel away from the launcher
  public void spinUp() {
    feederRoller.setVoltage(spinupFeederVolts.getDouble(SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(launchLauncherVolts.getDouble(LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }
  
  private static void setDefaultDouble(NetworkTableEntry entry, double defaultValue) {
    double cur = entry.getDouble(Double.NaN);

    if (Double.isNaN(cur))
      entry.setDouble(defaultValue);
  }

  public boolean launcherAtSpeed() {
    double rpm       = Math.abs(launcherEncoder.getVelocity()); // REV NEO velocity is in RPM. TODO: Confirm.
    double targetRPM = launcherAtSpeedRPM.getDouble(DEFAULT_AT_SPEED_RPM);
    double tolRPM    = launcherAtSpeedTolRPM.getDouble(DEFAULT_AT_SPEED_TOL_RPM);
    double debounceS = launcherAtSpeedDebounceS.getDouble(DEFAULT_AT_SPEED_DEBOUNCE_S);

    boolean inBand = rpm >= (targetRPM - tolRPM);

    if (inBand) {
      if (!atSpeedTimer.isRunning()) {
        atSpeedTimer.restart();
      }
    } else {
      atSpeedTimer.stop();
      atSpeedTimer.reset();
    }

    return atSpeedTimer.hasElapsed(debounceS);
  }

  // A command factory to turn the spinUp method into a command that requires this subsystem
  public Command spinUpCommand() {
    return this.runEnd(this::spinUp, this::stop);
  }

  // A command factory to turn the launch method into a command that requires this subsystem
  public Command launchCommand() {
    return this.runEnd(this::launch, this::stop);
  }

  // A command factory to turn the stop method into a command that requires this subsystem
  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  @Override
  public void periodic() {
    launcherRpmEntry.setDouble(launcherEncoder.getVelocity());
    atSpeedEntry.setBoolean(launcherAtSpeed());

    // SparkMax.getAppliedOutput() is [-1..1], multiply by bus voltage to approximate volts.
    feederAppliedVoltsEntry.setDouble(feederRoller.getAppliedOutput() * feederRoller.getBusVoltage());
    launcherAppliedVoltsEntry.setDouble(intakeLauncherRoller.getAppliedOutput() * intakeLauncherRoller.getBusVoltage());
  }
}
