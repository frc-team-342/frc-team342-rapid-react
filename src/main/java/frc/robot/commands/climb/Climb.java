// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends CommandBase {
  private ClimbSubsystem climb;
  private XboxController xbox;

  /** Creates a new StageOneClimb. */
  public Climb(ClimbSubsystem climb, XboxController xbox) {
    this.climb = climb;
    this.xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // deadbanding
    double liftSpeed = -MathUtil.applyDeadband(xbox.getLeftY(), 0.15);
    double rotateSpeed = MathUtil.applyDeadband(xbox.getRightX(), 0.15);

    climb.liftClimb(liftSpeed);
    climb.rotateClimb(rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimbLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
