// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {

  private MecanumDrive mecanumDrive;
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private boolean fieldOriented = true;
  private AHRS navx;

  /** Creates a new DriveSystem. */
  public DriveSystem() 
  {
    frontLeft = new CANSparkMax(1, MotorType.kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    navx = new AHRS();
  }

  public void drive(double xVelocity, double yVelocity, double rotationVelocity) {

    if (fieldOriented) 
    {
      mecanumDrive.driveCartesian(yVelocity, xVelocity, rotationVelocity, -navx.getAngle());
    } else {
      mecanumDrive.driveCartesian(yVelocity, xVelocity, rotationVelocity);
    }
  }

  public void toggleFieldOriented() {
    fieldOriented = !fieldOriented;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("drive").getEntry("fieldOriented").setBoolean(fieldOriented);
  }
}
