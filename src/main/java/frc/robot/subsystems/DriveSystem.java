// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {

  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private AHRS navx;

  private MecanumDrive mecanumDrive;
  private MecanumDriveOdometry odometry;

  private TrajectoryConfig trajectoryConfig;

  private double speedMultiplier = 0.8;

  private boolean fieldOriented = true;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    navx = new AHRS();
  }

  /**
   * Drives based on whether driving is field oriented or not
   * 
   * @param xVelocity velocity of the robot moving forward
   * @param yVelocity velocity of the robot moving side-to-side 
   * @param rotationVelocity velocity of robot moving clockwise 
   **/
  public void drive(double xVelocity, double yVelocity, double rotationVelocity) {
    // Used for slow mode 
    double x = xVelocity * speedMultiplier;
    double y = yVelocity * speedMultiplier;
    double rotation = rotationVelocity * speedMultiplier;

    if (fieldOriented) {
      mecanumDrive.driveCartesian(yVelocity, xVelocity, rotationVelocity, -navx.getAngle());
    } else {
      mecanumDrive.driveCartesian(yVelocity, xVelocity, rotationVelocity);
    }
  }

  private Pose2d getPose() {

  }

  /**
   * Generate a command for following a trajectory.
   * 
   * @param trajectory the trajectory to follow in the command
   * @param reversed whether the robot drives backwards during the trajectory or not
   * @return the command 
   */
  public MecanumControllerCommand trajectoryCommand(Trajectory trajectory, boolean reversed) {
    return new MecanumControllerCommand(
      trajectory, 
      pose, 
      kinematics, 
      new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), 
      thetaController, 
      desiredRotation, 
      maxWheelVelocityMetersPerSecond, 
      outputWheelSpeeds, 
      this
    );
  }

  public void toggleFieldOriented() {
    fieldOriented = !fieldOriented;
  }

  private boolean getFieldOriented() {
    return fieldOriented;
  }

  public void toggleSlowMode() {
    //If speedMultiplier is not on full speed, it sets it full speed and the inverse
    speedMultiplier = (speedMultiplier == 0.8) ? 0.4 : 0.8;
  }

  private double getSpeedMultiplier() {
    return speedMultiplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveSystem");
    builder.addBooleanProperty("Field Oriented", this::getFieldOriented, null);
    builder.addDoubleProperty("Speed Multiplier", this::getSpeedMultiplier, null);
  }
}
