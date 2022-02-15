// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import frc.robot.vision.Limelight;
import frc.robot.vision.PhotonVision;

import static frc.robot.Constants.ControllerConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSystem driveSystem;
  private IntakeSubsystem intake;
  private OuttakeSubsystem outtake;
  private ClimbSubsystem climb;

  private PhotonVision photon;
  private Limelight limelight;

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

  private XboxController operator;
  private JoystickButton deployBtn;
  private JoystickButton outtakeBtn;
  private JoystickButton stage2ForwardBtn;
  private JoystickButton stage2BackwardBtn;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Subsystems
    driveSystem = new DriveSystem();
    outtake = new OuttakeSubsystem();
    intake = new IntakeSubsystem();
    outtake = new OuttakeSubsystem();
    climb = new ClimbSubsystem();

    photon = new PhotonVision("camera");
    limelight = new Limelight();

    // Joystick
    driver = new Joystick(DRIVER_PORT);
    operator = new XboxController(OPERATOR_PORT);

    // Driver buttons
    toggleFieldOrientedBtn = new JoystickButton(driver, DRIVER_FIELD_ORIENTED_BTN); // Button 5
    toggleSlowModeBtn = new JoystickButton(driver, DRIVER_SLOW_MODE_BTN); // Button 7

    // Operator buttons
    deployBtn = new JoystickButton(operator, OP_DEPLOY_INTAKE_BTN); // Right bumper
    outtakeBtn = new JoystickButton(operator, OP_OUTTAKE_HIGH_BTN); // Left bumper
    stage2ForwardBtn = new JoystickButton(operator, OP_CLIMB_STAGE2_FORWARD_BTN); // X button
    stage2BackwardBtn = new JoystickButton(operator, OP_CLIMB_STAGE2_REVERSE_BTN); // Y button

    // Toggle Commands
    toggleFieldOriented = new InstantCommand(driveSystem::toggleFieldOriented, driveSystem);
    toggleSlowMode = new InstantCommand(driveSystem::toggleSlowMode, driveSystem);

    // Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    // Outtake Commands
    outtakeHigh = new OuttakeHigh(outtake);
    
    // Intake Commands
    deploy = new Deploy(intake);
    retract = new Retract(intake);
    intake.setDefaultCommand(retract);

    // Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    // Second stage climb commands
    stage2Backwards = new ClimbStageTwoBackward(climb);
    stage2Forwards = new ClimbStageTwoForward(climb);

    // Configure the button bindings
    configureButtonBindings();

    //Documentation for sendables: https://docs.wpilib.org/en/latest/docs/software/telemetry/robot-telemetry-with-sendable.html
    SmartDashboard.putData(driveSystem);
    SmartDashboard.putData(outtake);

    SmartDashboard.putData(limelight);
    SmartDashboard.putData(photon);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver
    toggleFieldOrientedBtn.whenPressed(toggleFieldOriented); // Button 5
    toggleSlowModeBtn.whenPressed(toggleSlowMode); // Button 7

    // Operator
    deployBtn.whileHeld(deploy); // Right bumper
    outtakeBtn.toggleWhenPressed(outtakeHigh); // Left bumper
    stage2ForwardBtn.whileHeld(stage2Forwards); // X button
    stage2BackwardBtn.whileHeld(stage2Backwards); // Y button
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

  /**
   * Test each subsystem with a test method and display the results on the dashboard
   */
  public void sendTestResults() {
    // individual subsystem test results
    Map<String, Boolean> driveResults = driveSystem.test();
    Map<String, Boolean> outtakeResults = outtake.test();
    Map<String, Boolean> intakeResults = intake.test();
    Map<String, Boolean> climbResults = climb.test();

    Map<String, Boolean> photonResults = photon.test();
    Map<String, Boolean> limelightResults = limelight.test();

    Map<String, Boolean> results = new HashMap<>();
    results.putAll(driveResults);
    results.putAll(outtakeResults);
    results.putAll(intakeResults);
    results.putAll(climbResults);
    results.putAll(photonResults);
    results.putAll(limelightResults);
    
    // name of every test failure
    String failures = "";

    // iterate over all results
    for(Map.Entry<String, Boolean> entry: results.entrySet()) {
      // test mode failures will be false
      if (entry.getValue().booleanValue() == false) {
        failures += entry.getKey() + ", ";
      }
    }
  
    // send results to dashboard
    SmartDashboard.putString("Test Mode Failures", failures);
  }
}
