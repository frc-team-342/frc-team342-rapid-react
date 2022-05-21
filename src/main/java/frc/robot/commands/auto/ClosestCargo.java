// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
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
  private boolean targetFound = false;

  /** Creates a new ClosestCargo. */
  public ClosestCargo(DriveSystem drive, IntakeSubsystem intake, OuttakeSubsystem outtake, Limelight limelight, PhotonVision photon) {
    this.drive = drive;
    this.intake = intake;
    this.outtake = outtake;

    this.limelight = limelight;
    this.photon = photon;
    
    addCommands(
      // find if cargo is visible, if so find angle
      new InstantCommand(() -> {
        targetFound = photon.hasTargets();

        // set angle if target found
        if (targetFound) {
          angle = photon.getHorizontalOffset();
        }
      }),

      // only run the rest of command if cargo is found
      new ConditionalCommand(
        // run if target is found
        new SequentialCommandGroup(
          // rotate to face cargo
          new RotateToAngle(drive, drive.getGyro() + angle)

          // drive to cargo position
          //drive.trajectoryCommand(generateTrajectory())
        ),

        // do nothing if not found
        new PrintCommand("Cargo not found"),

        // condition for running command
        () -> targetFound
      )
    );
  }

  private Trajectory generateTrajectory() {
    return TrajectoryGenerator.generateTrajectory(
      drive.getPose(), 
      List.of(), 
      drive.getPose().transformBy(photon.transformToTarget()), 
      DriveConstants.TRAJECTORY_CONFIG
    );
  }
}
