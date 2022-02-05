// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import javax.sound.midi.Sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.vision.Limelight;

public class DriveToHub extends SequentialCommandGroup {

  private DriveSystem driveSystem;
  private Limelight lime;

  private Trajectory trajectory;

  /** Creates a new DriveToHub. */
  public DriveToHub(DriveSystem driveSystem, Limelight lime) {
    this.driveSystem = driveSystem;
    this.lime = lime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);

    InstantCommand driveToHub = new InstantCommand(
      () -> {
        Transform2d transformToHub = lime.generateTransform();

        transformToHub = (transformToHub == null) ? new Transform2d() : transformToHub;

        Pose2d endPose = driveSystem.getPose().plus(transformToHub);

        trajectory = TrajectoryGenerator.generateTrajectory(
          driveSystem.getPose(), // current pose
          List.of(), // waypoints to hit along path
          endPose,  // desired end pose
          driveSystem.getTrajectoryConfig() // config includes max speed and accel
        );
      }, 
      driveSystem
    );

    addCommands(
      //Generated at runtime instead of in constructor - allows the robot to drive to hub
      driveToHub,

      //drives the instructed path
      driveSystem.trajectoryCommand(trajectory)
    );

  }

 
}
