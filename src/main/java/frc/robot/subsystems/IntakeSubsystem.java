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
import frc.robot.Robot;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX deployLeft;
  private WPI_TalonSRX deployRight;
  
  private WPI_TalonSRX rollerMotor;

  private double currentAngleLeft;
  private double currentAngleRight;

  private double intakeSpeed = INTAKE_SPEED;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // capital variable names are statically imported constants
    deployLeft = new WPI_TalonSRX(DEPLOY_LEFT_MOTOR);
    deployRight = new WPI_TalonSRX(DEPLOY_RIGHT_MOTOR);
    rollerMotor = new WPI_TalonSRX(ROLLER_MOTOR);

    // different inversions on A and B bot
    if (Robot.checkType() == Robot.RobotType.A_BOT) {
      rollerMotor.setInverted(true);
    }

    if (Robot.checkType() == Robot.RobotType.B_BOT) {
      // b bot is 9:1 gear ratio on rollers and a bot is 3:1
      intakeSpeed = 0.7;
    }

    deployRight.setInverted(true);

    deployLeft.configPeakCurrentLimit(DEPLOY_CURRENT_LIMIT);
    deployLeft.configPeakCurrentDuration(CURRENT_DURATION);
    deployRight.configPeakCurrentLimit(DEPLOY_CURRENT_LIMIT);
    deployRight.configPeakCurrentDuration(CURRENT_DURATION);

    // roller motor wiggling causes it to lose power and led to it burning out previously
    // lower current limit is necessary so that it doesnt stall and burn out
    rollerMotor.configPeakCurrentLimit(ROLLER_CURRENT_LIMIT);
    rollerMotor.configPeakCurrentDuration(CURRENT_DURATION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Multiplying by -360 to return a positive value. Supposed to be started in the fold position
    if (Robot.checkType() == Robot.RobotType.A_BOT) {
      currentAngleLeft = (deployLeft.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * -360.0f;
      currentAngleRight = (deployRight.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * -360.0f;
    } else {
      currentAngleLeft = (deployLeft.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * -360.0f;
      currentAngleRight = (deployRight.getSelectedSensorPosition() / ENCODER_TICKS_PER_ROTATION) * -360.0f;
    }
  }

  /**
   * Sets both encoders to 0 without a timeout. <br>
   * Timeout is not applied so that the reset does not fail when CAN frames are dropped on startup.
   */
  public void resetEncoders() {
    deployLeft.getSensorCollection().setQuadraturePosition(0, 0);
    deployRight.getSensorCollection().setQuadraturePosition(0, 0);
  }

  /**
   * Deploys the intake from the upper position into the intaking position. <br>
   * The maximum angle it deploys to is 52.75 degrees forwards from the starting position.
   * 
   * Adds more power to one of the motors if it is behind the other by multiplying the difference by a constant and adding it to output.
   */
  public void deployIntake() {
    if (currentAngleLeft >= MIN_INTAKE_ANGLE && currentAngleRight >= MIN_INTAKE_ANGLE) {
      deployLeft.set(0);
      deployRight.set(0);
    } else {
      if (isDeployLeftBehind()) {
        deployLeft.set((DEPLOY_SPEED) + (getDeltaDeployEncoders() * INTAKE_P));
        deployRight.set(DEPLOY_SPEED);
      } else if (isDeployRightBehind()) {
        deployLeft.set(DEPLOY_SPEED);
        deployRight.set((DEPLOY_SPEED) + (getDeltaDeployEncoders() * INTAKE_P));
      } else {
        deployLeft.set(DEPLOY_SPEED);
        deployRight.set(DEPLOY_SPEED);
      }
    }
  }

  /**
   * Retracts the intake from the intaking position above the bumpers. <br>
   * The minimum angle it retracts to is 15.0 degrees forwards from the starting position.
   * This is to avoid jamming the intake back into the original position.
   * 
   * Adds more power to one of the motors if it is behind the other by multiplying the difference by a constant and adding it to output.
   */
  public void retractIntake() {
    if (currentAngleLeft <= MAX_INTAKE_ANGLE && currentAngleRight <= MAX_INTAKE_ANGLE) {
      deployLeft.set(0);
      deployRight.set(0);
    } else {
      if (isDeployLeftBehind()) {
        deployLeft.set((DEPLOY_SPEED * -1) + (getDeltaDeployEncoders() * -1.0f * INTAKE_P));
        deployRight.set(DEPLOY_SPEED * -1);
      } else if (isDeployRightBehind()) {
        deployLeft.set(DEPLOY_SPEED * -1);
        deployRight.set((DEPLOY_SPEED * -1) + (getDeltaDeployEncoders() * -1.0f * INTAKE_P));
      } else {
        deployLeft.set(DEPLOY_SPEED * -1);
        deployRight.set(DEPLOY_SPEED * -1);
      }
    }
  }

  /**
   * Runs the intake rollers inwards to intake cargo
   */
  public void intakeCargo() {
    rollerMotor.set(intakeSpeed);
  }

  /**
   * Runs the intake rollers in reverse to eject cargo held below the conveyor
   */
  public void reverseIntakeCargo() {
    rollerMotor.set(intakeSpeed * -1);
  }

  /**
   * Stops the intake rollers
   */
  public void stopIntake() {
    rollerMotor.set(0);
  }

  /**
   * Returns current angle of the deployLeft encoder
   * 
   * @return degrees from starting position, positive is forwards
   */
  public double getCurrentAngleLeft() {
    return currentAngleLeft;
  }

  /**
   * Returns the current angle of the deployRight encoder
   * 
   * @return degrees from starting position, positive is forwards
   */
  public double getCurrentAngleRight() {
    return currentAngleRight;
  }

  /**
   * Gets the difference of the deploy encoder angles
   * 
   * @return absolute value of the difference
   */
  private double getDeltaDeployEncoders() {
    return Math.abs(currentAngleLeft - currentAngleRight);
  }

  /**
   * Checks to see if the left deploy encoder is less than the right deploy encoder, within a certain error margin
   * 
   * @return true if left encoder angle is less than right
   */
  private boolean isDeployLeftBehind() {
    return (currentAngleLeft < currentAngleRight - 0.1);
  }

  /**
   * Checks to see if the right deploy encoder is less than the left deploy encoder, within a certain error margin
   * 
   * @return true if right encoder angle is less than left
   */
  private boolean isDeployRightBehind() {
    return (currentAngleRight < currentAngleLeft - 0.1);
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
