// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.Deploy;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.drive.RotateToTarget;
import frc.robot.commands.outtake.OuttakeHigh;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.PhotonVision;


public class ShootThreeStart extends SequentialCommandGroup {
  private OuttakeSubsystem autoOuttake;
  private DriveSystem autoDriveSystem;
  private PhotonVision autoPhoton;
  private IntakeSubsystem autoIntake;
  private Limelight autoLime;
  
  /** Creates a new ShootThreeInAuto. */
  public ShootThreeStart(OuttakeSubsystem autoOuttake, DriveSystem autoDriveSystem, PhotonVision autoPhoton, IntakeSubsystem autoIntake, Limelight autoLime) {
    this.autoOuttake = autoOuttake;
    this.autoDriveSystem = autoDriveSystem;
    this.autoPhoton = autoPhoton;
    this.autoIntake = autoIntake;
    this.autoLime = autoLime;

    addRequirements(autoOuttake);
    addRequirements(autoDriveSystem);
    addRequirements(autoIntake);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    /**
     * This sequential command allows the robot to shoot the preloaded,
     * Collect the cargo from the tarmac and the terminal,
     * Return to the tarmac and drive to the hub,
     * And empties the two cargo into the upper hub.
     */
    addCommands(
      //Shoots the preloaded cargo into the upper port
      new OuttakeHigh(autoOuttake),
      //Backs up so the robot can target the next cargo
      new AutoDrive(autoDriveSystem, -0.8, 0.8),
      //Rotates the robot to a 60 degree angle
      new RotateToAngle(autoDriveSystem, 60.0),
      //Runs the DriveToCargo and Deploy commands at the same time, twice over
      new ParallelCommandGroup(new DriveToCargo(autoDriveSystem, autoPhoton), new Deploy(autoIntake)),
      new ParallelCommandGroup(new DriveToCargo(autoDriveSystem, autoPhoton), new Deploy(autoIntake)),
      //Drives the robot from the terminal to the tarmac
      new AutoDrive(autoDriveSystem, 0.8, 2.0),
      //Rotates the robot so it can target the hub
      new RotateToTarget(autoDriveSystem, autoLime, null),
      //Drives the robot to the hub
      new DriveToHub(autoDriveSystem, autoLime),
      //Outtakes the two collected cargo into the hub
      new OuttakeHigh(autoOuttake)
    );
  }
}
