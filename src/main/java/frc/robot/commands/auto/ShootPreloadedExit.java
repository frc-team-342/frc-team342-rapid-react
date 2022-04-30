// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.outtake.OuttakeHigh;
import frc.robot.commands.outtake.OuttakeLow;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.OuttakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPreloadedExit extends SequentialCommandGroup {

  private DriveSystem drive;
  private OuttakeSubsystem outtake;

  private AutoDrive exitTarmac;

  /** Creates a new DriveFunctions. */
  public ShootPreloadedExit(DriveSystem drive, OuttakeSubsystem outtake, boolean shootHigh) {
    this.drive = drive;
    this.outtake = outtake;

    exitTarmac = new AutoDrive(drive, 1.25);

    addRequirements(drive);
    addRequirements(outtake);
    
    addCommands(
      // applies the `withTimeout` to whichever is selected in the ternary expression
      (shootHigh ? new OuttakeHigh(outtake) : new OuttakeLow(outtake)).withTimeout(4),
      // if true: `new OuttakeHigh(outtake).withTimeout(4)`
      // if false: `new OuttakeLow(outtake).withTimeout(4)`

      exitTarmac
    );
  }
}
