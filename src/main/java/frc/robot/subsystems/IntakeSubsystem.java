// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX deployLeft;
  private WPI_TalonSRX deployRight;
  
  private WPI_TalonSRX rollerMotor;

  private double currentAngleLeft;
  private double currentAngleRight;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // capital variable names are statically imported constants
    deployLeft = new WPI_TalonSRX(DEPLOY_LEFT_MOTOR);
    deployRight = new WPI_TalonSRX(DEPLOY_RIGHT_MOTOR);

    deployRight.setInverted(true);

    deployLeft.configPeakCurrentLimit(CURRENT_LIMIT);
    deployLeft.configPeakCurrentDuration(CURRENT_DURATION);
    deployRight.configPeakCurrentLimit(CURRENT_LIMIT);
    deployRight.configPeakCurrentDuration(CURRENT_DURATION);

    rollerMotor = new WPI_TalonSRX(ROLLER_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngleLeft = (deployLeft.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * 360;
    currentAngleRight = (deployRight.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * 360;
  }

  /** 
  * Deploys the intake device for picking up cargo
  */
  public void deployIntake()
  {
    if(currentAngleLeft <= MIN_INTAKE_ANGLE && currentAngleRight <= MIN_INTAKE_ANGLE)
    {
      deployLeft.set(0);
      deployRight.set(0);
    }
    else
    {
      if (isDeployLeftBehind()) {
        deployLeft.set((DEPLOY_SPEED) + (getDeltaDeployEncoders() * INTAKE_P));
        deployRight.set(DEPLOY_SPEED);
      }
      else if (isDeployRightBehind()) {
        deployLeft.set(DEPLOY_SPEED);
        deployRight.set((DEPLOY_SPEED) + (getDeltaDeployEncoders() * INTAKE_P));
      }
      else {
        deployLeft.set(DEPLOY_SPEED);
        deployRight.set(DEPLOY_SPEED);
      }
    }
  }

  /** 
  * Retracts the intake device
  */
  public void retractIntake(){
  
    if(currentAngleLeft >= MAX_INTAKE_ANGLE && currentAngleRight >= MAX_INTAKE_ANGLE)
    {
      deployLeft.set(0);
      deployRight.set(0);
    }
    else
    {
      if (isDeployLeftBehind()) {
        deployLeft.set((DEPLOY_SPEED * -1) + (getDeltaDeployEncoders() * (INTAKE_P * -1)));
        deployRight.set(DEPLOY_SPEED * -1);
      }
      else if (isDeployRightBehind()) {
        deployLeft.set(DEPLOY_SPEED * -1);
        deployRight.set((DEPLOY_SPEED * -1) + (getDeltaDeployEncoders() * (INTAKE_P * -1)));
      } else {
        deployLeft.set(DEPLOY_SPEED * -1);
        deployRight.set(DEPLOY_SPEED * -1);
      }
    }
  }

  /**
   * Activates the intake rollers to collect cargo
   */
  public void intakeCargo()
  {
    rollerMotor.set(INTAKE_SPEED);
  }

  /**
   * Reverses the intake system to remove jammed cargo
   */
  public void reverseIntakeCargo()
  {
    rollerMotor.set(INTAKE_SPEED * -1);
  }

  /**
   * Stopes the intake rollers
   */
  public void stopIntake(){
    rollerMotor.set(0);
  }

  /**
   * Returns current angle of the deployLeft encoder
   * @return double value
   */
  public double getCurrentAngleLeft() {
    return currentAngleLeft;
  }

  /**
   * Returns the current angle of the deployRight encoder
   * @return double value
   */
  public double getCurrentAngleRight() {
    return currentAngleRight;
  }

  /**
   * Gets the absolute value of the difference of the Deploy encoder angles
   * @return double value
   */
  private double getDeltaDeployEncoders() {
    return Math.abs(currentAngleLeft - currentAngleRight);
  }

  /**
   * Checks to see if the left deploy encoder is less than the right deploy encoder
   * @return boolean value
   */
  private boolean isDeployLeftBehind() {
    if (currentAngleLeft < currentAngleRight - 0.1) {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Checks to see if the right deploy encoder is less than the left deploy encoder
   * @return boolean value
   */
  private boolean isDeployRightBehind() {
    if (currentAngleRight < currentAngleLeft - 0.1) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void initSendable(SendableBuilder sendable) {
    sendable.setSmartDashboardType("Intake");
    sendable.addDoubleProperty("Current Angle Left", this::getCurrentAngleLeft, null);
    sendable.addDoubleProperty("Current Angle Right", this::getCurrentAngleRight, null);
    sendable.addDoubleProperty("Delta Deploy Encoders", this::getDeltaDeployEncoders, null);
  }


  /**
   * Test that each motor controller is connected.
   * 
   * @return a map of the motor's name and a boolean with true if it is connected
   */
  public Map<String, Boolean> test() {
    var motors = new HashMap<String, Boolean>();

    deployLeft.getBusVoltage();
    motors.put("Deploy left motor", deployLeft.getLastError() == ErrorCode.OK);

    deployRight.getBusVoltage();
    motors.put("Deploy right motor", deployRight.getLastError() == ErrorCode.OK);

    rollerMotor.getBusVoltage();
    motors.put("Roller motor", rollerMotor.getLastError() == ErrorCode.OK);

    // still not 100% on this tbh
    motors.put("Deploy left encoder", deployLeft.getSensorCollection().getPulseWidthRiseToFallUs() != 0);
    motors.put("Deploy right encoder", deployRight.getSensorCollection().getPulseWidthRiseToFallUs() != 0);
    
    return motors;
  }
}
