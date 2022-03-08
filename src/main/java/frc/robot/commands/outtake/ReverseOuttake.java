// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.ShooterStates;

public class ReverseOuttake extends CommandBase {
  /** Creates a new ReverseOuttake. */
  public OuttakeSubsystem outtakeSub;

  public ReverseOuttake(OuttakeSubsystem outtakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.outtakeSub = outtakeSub;

    addRequirements(outtakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtakeSub.setState(ShooterStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    outtakeSub.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtakeSub.setState(ShooterStates.AUTOMATIC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
