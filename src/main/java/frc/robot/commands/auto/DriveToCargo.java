// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.vision.PhotonVision;

public class DriveToCargo extends SequentialCommandGroup {

  private DriveSystem subsystem;
  private PhotonVision photon;

  private Trajectory trajectory;

  /** Creates a new DriveToCargo. */
  public DriveToCargo(DriveSystem subsystem, PhotonVision photon) {
    this.subsystem = subsystem;
    this.photon = photon;

    addRequirements(subsystem);

    // inside a command so they are evaluated at runtime rather than instantiation
    InstantCommand generateTrajectory = new InstantCommand(
      () -> {
        // movement needed in order to get to cargo
        Transform2d transform = photon.transformToTarget();

        // if transform is null, instead do not move
        transform = (transform == null) ? new Transform2d() : transform;

        // desired end pose, generated from current location plus movement to target
        Pose2d endPose = subsystem.getPose().plus(transform);

        trajectory = TrajectoryGenerator.generateTrajectory(
          subsystem.getPose(), // current pose
          List.of(), // waypoints to hit along path
          endPose,  // desired end pose
          subsystem.getTrajectoryConfig() // config includes max speed and accel
        );
      },
      subsystem
    );

    addCommands(
      // generated at runtime instead of constructor
      generateTrajectory,
      // run generated path
      subsystem.trajectoryCommand(trajectory)
    );
  }

  
}
