// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.vision.Limelight;

public class RotateToTarget extends CommandBase {

  private DriveSystem drive;
  private Limelight lime;
  private Joystick joy;

  /** Creates a new RotateToTarget. */
  public RotateToTarget(DriveSystem driveSystem, Limelight limelight, Joystick j) {
    drive = driveSystem;
    lime = limelight;
    joy = j;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.setCamMode(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double deadBandX = MathUtil.applyDeadband(joy.getX(), 0.15);
    double deadBandY = MathUtil.applyDeadband(joy.getY(), 0.15);

    drive.driveWithTargeting(deadBandX, deadBandY, lime.getHorizontalOffset());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
