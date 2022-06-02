// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.intake.Deploy;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.PhotonVision;

/** Rotates to the closest cargo, picks it up, turns around, and shoots it */
public class ClosestCargo extends SequentialCommandGroup {
  private DriveSystem drive;
  private IntakeSubsystem intake;

  private Limelight limelight;
  private PhotonVision photon;
  
  private double angle = 0;
  private boolean targetFound = false;

  /** Creates a new ClosestCargo. */
  public ClosestCargo(DriveSystem drive, IntakeSubsystem intake, Limelight limelight, PhotonVision photon) {
    this.drive = drive;
    this.intake = intake;

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
          new RotateToAngle(drive, drive.getGyro() + angle),

          // drive to cargo position
          new RunCommand(() -> {
            // drive towards cargo
            drive.driveWithTargeting(0.3, 0, angle);
          }, drive).raceWith(
            // drive until within distance
            new WaitUntilCommand(() -> {
              // distance is an meters
              return photon.getDistance() <= 0.2;
            })
          ).andThen(() -> {
            // stop driving
            drive.setBrakeMode(true);
            drive.drive(0, 0, 0);
          }),

          // intake only works on B bot
          new ConditionalCommand(
            new InstantCommand(), 
            new Deploy(intake).withTimeout(2.0),
            // check for which robot code is running on
            () -> Robot.checkType() == Robot.RobotType.B_BOT
          )
        ),

        // do nothing if not found
        new PrintCommand("Cargo not found"),

        // condition for running command
        () -> targetFound
      )
    );
  }
}
