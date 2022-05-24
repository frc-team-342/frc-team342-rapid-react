// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.outtake.OuttakeHigh;
import frc.robot.commands.outtake.OuttakeLow;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {

  private DriveSystem drive;
  private OuttakeSubsystem outtake;
  private IntakeSubsystem intake;

  private AutoDrive exitTarmac;

  /** Creates a new DriveFunctions. */
  public TwoBallAuto(DriveSystem drive, OuttakeSubsystem outtake, IntakeSubsystem intake, boolean shootHigh) {
    this.drive = drive;
    this.outtake = outtake;
    this.intake = intake;

    exitTarmac = new AutoDrive(drive, 1.25);

    addRequirements(drive);
    addRequirements(outtake);
    
    addCommands(
      // shoot preloaded
      (shootHigh ? new OuttakeHigh(outtake) : new OuttakeLow(outtake)).withTimeout(4),
      
      // drive to next power cell while intaking
      exitTarmac.raceWith(new Intake(intake)),

      // turn around after picked up
      new RotateToAngle(drive, 180),

      // drive back to hub
      new AutoDrive(drive, 1.2),

      // shoot again
      (shootHigh ? new OuttakeHigh(outtake) : new OuttakeLow(outtake)).withTimeout(4)
    );
  }
}

