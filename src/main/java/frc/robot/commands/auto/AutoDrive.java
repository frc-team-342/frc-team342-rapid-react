// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class AutoDrive extends CommandBase {

  private DriveSystem subsystem;
  private Timer timer;

  private final double seconds;
  private double speed = 0.6;

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveSystem subsystem, double seconds) {
    this.subsystem = subsystem;
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subsystem);
    timer = new Timer();
  }

  public AutoDrive(DriveSystem subsystem, double speed, double seconds) {
    this.subsystem = subsystem;
    this.seconds = seconds;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subsystem);
    timer = new Timer();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // set brake mode so it does not continue coasting after auto
    subsystem.setBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.drive(0, speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}
