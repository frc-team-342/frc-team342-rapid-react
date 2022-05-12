// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

import static frc.robot.Constants.DriveConstants.*;

public class DriveVelocity extends CommandBase {
  private DriveSystem driveSystem;
  private Joystick joystick;

  /** Creates a new DriveVelocity. */
  public DriveVelocity(DriveSystem driveSystem, Joystick joystick) {
    this.driveSystem = driveSystem;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = joystick.getX() * MAX_X_SPEED;
    double yVel = joystick.getY() * MAX_Y_SPEED;
    double rotVel = joystick.getZ() * MAX_ROTATION_SPEED;

    driveSystem.driveVelocity(xVel, yVel, rotVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
