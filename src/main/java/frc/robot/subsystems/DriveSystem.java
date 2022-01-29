// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {

  private MecanumDrive mecanumDrive;
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private double speedMultiplier = 0.8;

  private boolean fieldOriented = true;
  private ADIS16470_IMU gyro;

  /** Creates a new DriveSystem. */
  public DriveSystem() 
  {
    frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    gyro = new ADIS16470_IMU();
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

  /**
   * 
   * @param targetAngle takes an angle between 0-360 degrees
   * This method rotates clockwise if targetAngle is between 0 and 180 degrees, and rotates counterclockwise if targetAngle is between 181 and 360 degrees.
   * It also updates the currentAngle variable to the new angle after rotating.
   */
  public void rotateToAngle(double targetAngle)
  {
    double desiredAngle = targetAngle;
    double currentAngle = gyro.getAngle();
    
    if(currentAngle != (desiredAngle - 5) || currentAngle != (desiredAngle + 5))
    {
        if(desiredAngle <= 180)
      {
        mecanumDrive.driveCartesian(0, 0, 0.4);
      }
      else if(desiredAngle > 180)
      {
      mecanumDrive.driveCartesian(0, 0, -0.4);
      }
    }

    currentAngle = gyro.getAngle();
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
