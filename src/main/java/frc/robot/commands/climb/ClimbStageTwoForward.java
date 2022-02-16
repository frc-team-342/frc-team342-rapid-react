// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbStageTwoForward extends CommandBase {
  
  private ClimbSubsystem stage2Subsystem;

  public ClimbStageTwoForward(ClimbSubsystem subsystem) {
    stage2Subsystem = subsystem;
    addRequirements(stage2Subsystem);
  }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stage2Subsystem.stage2RotateForwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage2Subsystem.deactivateStage2();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
