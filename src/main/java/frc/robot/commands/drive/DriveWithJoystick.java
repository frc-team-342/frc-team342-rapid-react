// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class DriveWithJoystick extends CommandBase {
  
  DriveSystem driveSystem;
  Joystick driver;
  
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(DriveSystem drive, Joystick j) {

    addRequirements(drive);
    this.driveSystem = drive;
    this.driver = j;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Checks whether joystick is within a deadzone and returns val
    double deadBandX = MathUtil.applyDeadband(driver.getX(), 0.15);
    double deadBandY = MathUtil.applyDeadband(driver.getY(), 0.15);
    double deadBandZ = MathUtil.applyDeadband(driver.getZ(), 0.15);

    driveSystem.drive(deadBandX, deadBandY, deadBandZ);

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
