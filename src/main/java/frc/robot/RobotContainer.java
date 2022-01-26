// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveWithJoystick driveWithJoystick;

  private DriveSystem driveSystem;

  private IntakeSubsystem intakeSystem;

  private InstantCommand retractIntakeCommand;

  private InstantCommand extendIntakeCommand;

  private InstantCommand intakeCargoCommand;

  private InstantCommand reverseIntakeCommand;

  private Joystick driverController;

  private JoystickButton reverseIntakeButton;

  private JoystickButton extendIntakeButton;

  private JoystickButton intakeCargoButton;

  private JoystickButton retractIntakeButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    retractIntakeCommand = new InstantCommand(intakeSystem::retractIntake, intakeSystem);
    extendIntakeCommand = new InstantCommand(intakeSystem::deployIntake, intakeSystem);
    intakeCargoCommand = new InstantCommand(intakeSystem::activateIntake, intakeSystem);
    reverseIntakeCommand = new InstantCommand(intakeSystem::reverseIntake, intakeSystem);

    reverseIntakeButton = new JoystickButton(driverController, 1);
    extendIntakeButton = new JoystickButton(driverController, 2);
    intakeCargoButton = new JoystickButton(driverController, 3);
    retractIntakeButton = new JoystickButton(driverController, 4);


    
    // Configure the button bindings
    configureButtonBindings();

  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    reverseIntakeButton.whileHeld(reverseIntakeCommand);
    intakeCargoButton.whileHeld(intakeCargoCommand);
    extendIntakeButton.whenHeld(extendIntakeCommand);
    retractIntakeButton.whenHeld(retractIntakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
