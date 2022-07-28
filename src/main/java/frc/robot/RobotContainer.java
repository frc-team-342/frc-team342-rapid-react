// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intake.Deploy;
import frc.robot.commands.intake.ManualUptake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.auto.ShootPreloadedExit;
import frc.robot.commands.auto.ShootThreeStart;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.TurnOnClimbMode;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.outtake.OuttakeHigh;
import frc.robot.commands.outtake.OuttakeLow;
import frc.robot.commands.intake.ReverseAll;
import frc.robot.commands.outtake.ReverseOuttake;
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
  private InstantCommand setFieldOriented;
  private InstantCommand toggleSlowMode;
  private InstantCommand toggleTurboMode;
  private InstantCommand resetIntakeEncoders;
  private InstantCommand toggleLimelight;

  private Command deploy;
  private Command retract;
  //private Command intakeCmd; // Used for testing the rollers without deploying the intake
  private Command reverseIntake;

  private Command climbCmd;
  private Command climbModeEnable;
  private Command reverseAll;
  private Command reverseOuttake;
  private Command manualUptake;

  private DriveWithJoystick driveWithJoystick;

  private OuttakeHigh outtakeHigh;
  private OuttakeLow outtakeLow;
  
  // Driver Buttons
  private Joystick driver;
  private JoystickButton driver_toggleLimelightBtn;
  private JoystickButton driver_toggleFieldOrientedBtn;
  private JoystickButton driver_toggleSlowModeBtn;
  private JoystickButton driver_outtakeHighBtn;
  private JoystickButton driver_outtakeLowBtn;

  // Operator Buttons
  private XboxController operator;
  private JoystickButton op_deployBtn;
  private JoystickButton op_reverseOuttakeBtn;
  private JoystickButton op_climbModeBtn;
  private JoystickButton op_toggleSlowModeBtn;
  private JoystickButton op_outtakeHighBtn;
  private JoystickButton op_reverseAllBtn;
  private JoystickButton op_manualUptakeBtn;
  private Trigger op_outtakeLowBtn;

  private SendableChooser<Command> autoChooser;
  private Command driveToCargo;
  private Command driveToHub;
  private Command shootThreeStart;
  private Command shootPreloadHigh;
  private Command shootPreloadLow;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Subsystems
    driveSystem = new DriveSystem();
    outtake = new OuttakeSubsystem();
    intake = new IntakeSubsystem();
    climb = new ClimbSubsystem();

    photon = new PhotonVision("camera");
    limelight = new Limelight();

    // Joystick
    driver = new Joystick(DRIVER_PORT);
    operator = new XboxController(OPERATOR_PORT);

    // Driver buttons
    driver_toggleLimelightBtn = new JoystickButton(driver, DRIVER_TOGGLE_LIMELIGHT_BTN); // Button 1
    driver_toggleFieldOrientedBtn = new JoystickButton(driver, DRIVER_FIELD_ORIENTED_BTN); // Button 6
    driver_toggleSlowModeBtn = new JoystickButton(driver, DRIVER_SLOW_MODE_BTN); // Button 4
    driver_outtakeLowBtn = new JoystickButton(driver, DRIVER_OUTTAKE_LOW_BTN);
    driver_outtakeHighBtn = new JoystickButton(driver, DRIVER_OUTTAKE_HIGH_BTN);

    // Operator buttons
    op_deployBtn = new JoystickButton(operator, OP_DEPLOY_INTAKE_BTN); // Right bumper
    op_outtakeHighBtn = new JoystickButton(operator, OP_OUTTAKE_HIGH_BTN); // Left bumper
    op_outtakeLowBtn = new Trigger(() -> { return (operator.getLeftTriggerAxis() >= 0.8); }); // Left trigger
    op_reverseOuttakeBtn = new JoystickButton(operator, OP_REVERSE_OUTTAKE_BTN); // B Button
    op_climbModeBtn = new JoystickButton(operator, OP_CLIMB_MODE_BTN); // Y button
    op_reverseAllBtn = new JoystickButton(operator, OP_REVERSE_ALL_BTN); // X button
    op_toggleSlowModeBtn = new JoystickButton(operator, OP_TOGGLE_SLOW_BTN); // Back/Select Button
    op_manualUptakeBtn = new JoystickButton(operator, OP_MANUAL_UPTAKE_BTN); // A button

    // Toggle Commands
    toggleFieldOriented = new InstantCommand(driveSystem::toggleFieldOriented, driveSystem);
    toggleSlowMode = new InstantCommand(driveSystem::toggleSlowMode, driveSystem);
    toggleTurboMode = new InstantCommand(driveSystem::toggleTurboMode, driveSystem);
    resetIntakeEncoders = new InstantCommand(intake::resetEncoders, intake);
    toggleLimelight = new InstantCommand(limelight::toggleDriverMode);

    // Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    // Outtake Commands
    outtakeHigh = new OuttakeHigh(outtake);
    outtakeLow = new OuttakeLow(outtake);
    
    // Intake Commands
    retract = new Retract(intake, outtake);
    deploy = new Deploy(intake);
    reverseAll = new ReverseAll(intake, outtake);
    reverseOuttake = new ReverseOuttake(outtake);
    //intakeCmd = new Intake(intake);
    reverseIntake = new ReverseIntake(intake);
    manualUptake = new ManualUptake(outtake);

    intake.setDefaultCommand(retract);

    // Drive With Joystick
    driveWithJoystick = new DriveWithJoystick(driveSystem, driver);
    driveSystem.setDefaultCommand(driveWithJoystick);

    // Climb
    climbCmd = new Climb(climb, operator);
    climbModeEnable = new TurnOnClimbMode(climb, intake);
    climb.setDefaultCommand(climbCmd);

    climb.resetLiftEncoders();

    // Configure the button bindings
    configureButtonBindings();

    //Documentation for sendables: https://docs.wpilib.org/en/latest/docs/software/telemetry/robot-telemetry-with-sendable.html
    SmartDashboard.putData(driveSystem);
    SmartDashboard.putData(outtake);
    SmartDashboard.putData(limelight);
    SmartDashboard.putData(photon);
    SmartDashboard.putData(climb);
    SmartDashboard.putData(intake);

    // Makes the autonomous chooser and associated commands
    autoChooser = new SendableChooser<>();
    shootThreeStart = new ShootThreeStart(outtake, driveSystem, photon, intake, limelight);

    shootPreloadHigh = new ShootPreloadedExit(driveSystem, outtake, true);
    shootPreloadLow = new ShootPreloadedExit(driveSystem, outtake, false);

    // Add options to the smart dashboard
    autoChooser.setDefaultOption("Shoot preloaded high and exit tarmac", shootPreloadHigh);
    autoChooser.setDefaultOption("Shoot preloaded low and exit tarmac", shootPreloadLow);

    // Sets chooser name and sends to dashboard
    SendableRegistry.setName(autoChooser, "Autonomous Chooser");
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver
    driver_toggleLimelightBtn.whenPressed(toggleLimelight); // Button 1
    driver_toggleFieldOrientedBtn.whenPressed(toggleFieldOriented); // Button 6
    driver_toggleSlowModeBtn.whenPressed(toggleSlowMode); // Button 4
    driver_outtakeLowBtn.whileHeld(outtakeLow); // Button 8
    driver_outtakeHighBtn.whileHeld(outtakeHigh); // Button 3

    // Operator
    op_toggleSlowModeBtn.whenPressed(toggleSlowMode); // Select button
    op_climbModeBtn.whenPressed(climbModeEnable); // Y button
    op_deployBtn.whileHeld(deploy); // Right bumper
    op_outtakeHighBtn.whileHeld(outtakeHigh); // Left bumper
    op_reverseOuttakeBtn.whileHeld(reverseOuttake); // B button
    op_outtakeLowBtn.whileActiveContinuous(outtakeLow); // Right trigger
    op_reverseAllBtn.whileHeld(reverseAll); // X button
    op_manualUptakeBtn.whileHeld(manualUptake); // A button
  }

  public void resetIntakeEncoders() {
    intake.resetEncoders();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void teleopCoastMode() {
    driveSystem.setBrakeMode(false);
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
