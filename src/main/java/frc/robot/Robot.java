// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;


import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public enum RobotType {A_BOT, B_BOT}

  private static AnalogInput robotCheckAnalog = new AnalogInput(Constants.ROBOT_ANALOG_CHECKER_CHANNEL);
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // directory where git data is stored
    File deployDir = Filesystem.getDeployDirectory();

    // files where git data is stored
    File branchFile = new File(deployDir, "branch.txt");
    File commitFile = new File(deployDir, "commit.txt");

    String branch = "";
    String commit = "";

    // check that file exists before access
    if (branchFile.exists()) {
      try {
        // convert file to string
        branch = new String(Files.readAllBytes(branchFile.toPath())).trim();
      } catch (IOException e) {
        // readAllBytes throws an IOException
        System.err.println(e);
        branch = "branch not found";
      }
    }

    if (commitFile.exists()) {
      try {
        // convert file to string
        commit = new String(Files.readAllBytes(commitFile.toPath())).trim();
      } catch (IOException e) {
        // readAllBytes throws an IOException
        System.err.println(e);
        commit = "commit not found";
      }
    }
    
    // send to dashboard
    SmartDashboard.putString("Git Branch", branch);
    SmartDashboard.putString("Git Commit", commit);

    SmartDashboard.putString("Current Detected Robot", (checkType() == RobotType.A_BOT) ? "A Robot" : "B Robot");

    // reset encoders when rio restarts
    m_robotContainer.resetIntakeEncoders();
  }

  /**
   * This method allows us to check what robot is being used. <br>
   * There is a jumper on analog input 0 on b bot that makes it always return voltage.
   * 
   * @return the robot being used
   */
  public static RobotType checkType(){
    if(robotCheckAnalog.getVoltage() < Constants.VOLTAGE_THRESHOLD){
      return RobotType.A_BOT;
    } else {
      return RobotType.B_BOT;
    }
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.sendTestResults();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
