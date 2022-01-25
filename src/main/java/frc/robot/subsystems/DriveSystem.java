// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {

  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder backRightEncoder;

  private MecanumDrive mecanumDrive;
  private MecanumDriveOdometry odometry;

  private TrajectoryConfig trajectoryConfig;

  private double speedMultiplier = 0.8;

  private boolean fieldOriented = true;
  private ADXRS450_Gyro gyro;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    gyro = new ADXRS450_Gyro();
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

    if (fieldOriented) 
    {
      mecanumDrive.driveCartesian(y, x, rotation, -gyro.getAngle());
    } else {
      mecanumDrive.driveCartesian(y, x, rotation);
    }
  }

  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Get the current speeds of the wheel as a MecanumDriveWheelSpeeds object. <br/>
   * Units are RPM.
   * 
   * @return the current wheel speeds
   */
  private MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity(),
      backLeftEncoder.getVelocity(),
      frontRightEncoder.getVelocity(),
      backRightEncoder.getVelocity()
    );
  }

  private void driveVolts(MecanumDriveMotorVoltages voltages) {
    // "yeah can i get a uhhh MecanumDriveMotorVoltages"
    // ^^ statements dreamed up by the utterly DERANGED
    frontLeft.setVoltage(voltages.frontLeftVoltage);
    backLeft.setVoltage(voltages.rearLeftVoltage);
    frontRight.setVoltage(voltages.frontRightVoltage);
    backRight.setVoltage(voltages.rearRightVoltage);
  }

  /**
   * Generate a command for following a trajectory.
   * 
   * @param trajectory the trajectory to follow in the command
   * @param reversed whether the robot drives backwards during the trajectory or not
   * @return the command that follows the path
   */
  public MecanumControllerCommand trajectoryCommand(Trajectory trajectory, boolean reversed) {
    return new MecanumControllerCommand(
      trajectory, // Path to follow
      this::getPose, // Current robot position

      Constants.DriveConstants.FEEDFORWARD, // Feedforward control
      Constants.DriveConstants.KINEMATICS, // Distance from center of robot to each wheel

      new PIDController(0, 0, 0), // PID controller on x-position
      new PIDController(0, 0, 0), // PID controller on y-position
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)), // PID controller on rotation

      0, // Maximum speed in m/s

      new PIDController(0, 0, 0), // Front left velocity controller
      new PIDController(0, 0, 0), // Back left velocity controller
      new PIDController(0, 0, 0), // Front right velocity controller
      new PIDController(0, 0, 0), // Back right velocity controller

      this::getWheelSpeeds, // Method pointer to getter for current wheel speeds
      this::driveVolts, // Method pointer to voltage output
      this // Command dependencies
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
