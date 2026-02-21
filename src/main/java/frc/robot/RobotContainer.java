// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.CANFuelSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem;
  //private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  // The operator's controller
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // The autonomous chooser
  // NOTE: this is not currently used since we don't have any autonomous commands, but it is set up to be easily added to in the future
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private double speedScale = 1.0; // Default speed scale (100%)

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize the swerve subsystem with the deploy directory
    m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    
    // Configure the trigger bindings
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with autoChooser.addOption
    // NOTE: Need to investigate what this does and how to use it, since we don't have any autonomous commands yet. It may be useful for testing, though.
    //autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(m_driverController, ballSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // *********  OPERATOR CONTROLS  *********
    // LEFT BUMPER: INTAKE
    // While the left bumper on operator controller is held, intake Fuel at the default speed. When the button is released, stop.
    //m_operatorController.leftBumper()
    //    .whileTrue(m_intakeSubsystem.runEnd(() -> m_intakeSubsystem.intakeCommand(), () -> m_intakeSubsystem.stop()));
    m_operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));

    // While the left bumper on driver controller is held, intake Fuel at 50% speed. When the button is released, stop.
    //m_driverController.leftBumper()
    //    .whileTrue(m_intakeSubsystem.runEnd(() -> m_intakeSubsystem.setSpeed(0.5), () -> m_intakeSubsystem.stop()));
    
    // RIGHT BUMPER: SHOOT
    // While the right bumper on the operator controller is held, launch fuel at full speed. When the button is released, stop.
    //m_operatorController.rightBumper()
    //    .whileTrue(m_shooterSubsystem.runEnd(() -> m_shooterSubsystem.shootCommand(), () -> m_shooterSubsystem.stop()));
    
    // While the right bumper on the operator controller is held, spin up for 1 second, then launch fuel. When the button is released, stop.
    m_operatorController.rightBumper()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(Constants.ShooterConstants.SPIN_UP_SECONDS)
        .andThen(ballSubsystem.launchCommand())
        .finallyDo(() -> ballSubsystem.stop()));

    // A BUTTON: REVERSE INTAKE (EJECT)
    // While the A button is held on the operator controller, eject fuel back out the intake
    //m_operatorController.a()
    //    .whileTrue(m_intakeSubsystem.runEnd(() -> m_intakeSubsystem.reverseIntakeCommand(), () -> m_intakeSubsystem.stop()));
    m_operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    // ************************************

    // *********  DRIVE CONTROLS  *********
    // START BUTTON: ZERO GYRO
    // Press the start button on the driver controller to zero the gyro on the swerve drive. This is important for field-oriented control.
    m_driverController.start()
        .onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.zeroGyro()));

    // A BUTTON: TOGGLE SPEED SCALE (slow mode)
    // Press the A button on the driver controller to toggle between full speed and half speed.
    m_driverController.a()
        .onTrue(new InstantCommand(() -> speedScale = 0.5))
        .onFalse(new InstantCommand(() -> speedScale = 1.0));

    // DRIVE COMMANDS
    // TESTING
    // Robot-centric drive command for testing and tuning the swerve drive.
    // Not field-oriented. Controls are relative to the robot's current orientation. 
    // Drive the robot with the left stick for translation and the right stick for rotation. Useful for testing and tuning the swerve drive
    Command driveFieldOrientedAngVel = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.15));

    // COMPETITION
    // Drive the robot with the left stick for translation and the right stick for rotation. This is field-oriented, so the
    // controls are relative to the field rather than the robot's current orientation. This is generally more intuitive for drivers,
    // but may require more tuning to get right.

    // BASE Drive command with speed scaling applied to translation, but not rotation.
    /* Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * speedScale,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale */

    // SQUARED Drive command with speed scaling applied to translation, but not rotation and with squared joystick input shaping.
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -shapeAxis(MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)) * speedScale,
        () -> -shapeAxis(MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale

    // EXPO Drive command with speed scaling applied to translation, but not rotation and with exponential joystick input shaping.
    /* Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -expo(MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), DriverConstants.TRANSLATION_EXPO) * speedScale,
        () -> -expo(MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), DriverConstants.TRANSLATION_EXPO) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale */

    //m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngVel);
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return autoChooser.getSelected();

    // This creates a sequence: Drive forward, wait 1 second, then drive back.
    return new SequentialCommandGroup(
      new DriveDistance(1, m_swerveSubsystem),  // Move 1 meter
      new WaitCommand(1.0),                    // Pause
      new DriveDistance(-1, m_swerveSubsystem)         // Return
    );
  }

  // Square or Cube the input while preserving the sign, to give finer control at low speeds. Joystick input shaping.
  private static double shapeAxis(double value) {
    // value is expected in [-1, 1]
    // Square
    return Math.copySign(value * value, value);

    // Cube
    // return value * value * value;
  }

  // Exponential shaping of the input, with a tunable exponent. Joystick input shaping.
  private static double expo(double x, double expo) {
    // expo in [0..1]. 0 = linear, 1 = very soft near center
    return (1 - expo) * x + expo * x * x * x;
  }
}
