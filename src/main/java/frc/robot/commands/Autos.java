// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.thethriftybot.server.CAN;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;

public final class Autos {
  /** Static factory for an autonomous command. */
  public static Command exampleAuto(CANFuelSubsystem subsystem) {
    return Commands.sequence(
      subsystem.spinUpCommand().withTimeout(0.5),
      subsystem.launchCommand().withTimeout(0.75),
      subsystem.stopCommand()
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
