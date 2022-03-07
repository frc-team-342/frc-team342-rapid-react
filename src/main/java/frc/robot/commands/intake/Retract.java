// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem.ShooterStates;

public class Retract extends CommandBase {
  /** Creates a new Retract. */
  private IntakeSubsystem intake;
  private OuttakeSubsystem outtakeSub;
  public Retract(IntakeSubsystem intake, OuttakeSubsystem outtakeSub) {

    this.intake = intake;
    this.outtakeSub = outtakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, outtakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outtakeSub.setState(ShooterStates.AUTOMATIC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.retractIntake();
    intake.stopIntake();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
