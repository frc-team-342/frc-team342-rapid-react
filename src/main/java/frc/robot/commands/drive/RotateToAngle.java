// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class RotateToAngle extends CommandBase {
  private DriveSystem rotateToAngleDrive;
  private double targetAngle;
  
  /** Creates a new RotateToAngle. */
  public RotateToAngle(DriveSystem rotateToAngleDrive, double targetAngle) {
    this.rotateToAngleDrive = rotateToAngleDrive;
    this.targetAngle = targetAngle;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotateToAngleDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotateToAngleDrive.rotateToAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
