// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.Deploy;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.climb.ClimbStageTwoBackward;
import frc.robot.commands.climb.ClimbStageTwoForward;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.outtake.OuttakeHigh;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.OuttakeSubsystem;

import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSystem driveSystem;
  private OuttakeSubsystem outtake;
  private IntakeSubsystem intake;
  private ClimbSubsystem climb;


  private InstantCommand toggleFieldOriented; 
  private InstantCommand toggleSlowMode;
  private Command deploy;
  private Command retract;
  private Command stage2Backwards;
  private Command stage2Forwards;

  private DriveWithJoystick driveWithJoystick;

  private OuttakeHigh outtakeHigh;

  private Joystick driver;
  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleSlowModeBtn;
  private JoystickButton deployButton;
  private JoystickButton stage2ForwardButton;
  private JoystickButton stage2BackwardButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Subsystems
    driveSystem = new DriveSystem();
    outtake = new OuttakeSubsystem();
    intake = new IntakeSubsystem();
    climb = new ClimbSubsystem();

    //Joystick
    driver = new Joystick(0);

    //Buttons
    toggleFieldOrientedBtn = new JoystickButton(driver, 5); // left trigger
    toggleSlowModeBtn = new JoystickButton(driver, 7); // Select Button
    deployButton = new JoystickButton(driver, 6); // right trigger
    stage2ForwardButton = new JoystickButton(driver, 4); // Y button
    stage2BackwardButton = new JoystickButton(driver, 3); // X button
    


    //Commands 
    //Toggle Commands
    toggleFieldOriented = new InstantCommand(driveSystem::toggleFieldOriented, driveSystem);
    toggleSlowMode = new InstantCommand(driveSystem::toggleSlowMode, driveSystem);

    //Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    //Outtake Commands
    outtakeHigh = new OuttakeHigh(outtake);
    
    //Intake Commands
    deploy = new Deploy(intake);
    retract = new Retract(intake);
    intake.setDefaultCommand(retract);

    //Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    //Second stage climb commands
    stage2Backwards = new ClimbStageTwoBackward(climb);
    stage2Forwards = new ClimbStageTwoForward(climb);

    // Configure the button bindings
    configureButtonBindings();

    //Documentation for sendables: https://docs.wpilib.org/en/latest/docs/software/telemetry/robot-telemetry-with-sendable.html
    SmartDashboard.putData(driveSystem);
    SmartDashboard.putData(outtake);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleFieldOrientedBtn.whenPressed(toggleFieldOriented);
    toggleSlowModeBtn.whenPressed(toggleSlowMode);
    deployButton.whileHeld(deploy);
    stage2ForwardButton.whileHeld(stage2Forwards);
    stage2BackwardButton.whileHeld(stage2Backwards);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // trajectory to follow during auto
    var trajectory = new Trajectory();

    // command to follow that trajectory
    return driveSystem.trajectoryCommand(trajectory);
  }
}
