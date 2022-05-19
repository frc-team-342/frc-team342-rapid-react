// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.PhotonVision;

/** Rotates to the closest cargo, picks it up, turns around, and shoots it */
public class ClosestCargo extends SequentialCommandGroup {
  private DriveSystem drive;
  private IntakeSubsystem intake;
  private OuttakeSubsystem outtake;

  private Limelight limelight;
  private PhotonVision photon;
  
  private double angle = 0;

  /** Creates a new ClosestCargo. */
  public ClosestCargo(DriveSystem drive, IntakeSubsystem intake, OuttakeSubsystem outtake, Limelight limelight, PhotonVision photon) {
    this.drive = drive;
    this.intake = intake;
    this.outtake = outtake;

    this.limelight = limelight;
    this.photon = photon;
    
    addCommands(
      // find angle of cargo, NaN if not present
      new InstantCommand(() -> {
        if (photon.hasTargets()) {
          angle = photon.getHorizontalOffset();
        } else {
          // do not rotate if target isnt found
          angle = 0;
        }
      }),

      // rotate to face the cargo
      new RotateToAngle(drive, drive.getGyro() + angle),

      // drive to it?? pid 
      new AutoDrive(drive, 0.0, 0.0)
    );
  }
}
