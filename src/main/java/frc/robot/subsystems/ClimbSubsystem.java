// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.ClimbConstants.*;



public class ClimbSubsystem extends SubsystemBase {

  private WPI_TalonFX climbMotor1;
  private WPI_TalonFX climbMotor2;
  private WPI_TalonSRX secondStageMotor1;
  private WPI_TalonSRX secondStageMotor2;

  private TalonFXSensorCollection leftLiftEncoder;
  private TalonFXSensorCollection rightLiftEncoder;
  
  //Sets the second stage climb initial angle using the rotation ticks from the hex bore encoder
  private double secondStageInitialAngle = (2048 / 8192) * 360;

  private double currentAngle;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor1 = new WPI_TalonFX(CLIMB_LEFT_MOTOR);
    climbMotor2 = new WPI_TalonFX(CLIMB_RIGHT_MOTOR);

    secondStageMotor1 = new WPI_TalonSRX(CLIMB_SECOND_MOTOR_1);
    secondStageMotor2 = new WPI_TalonSRX(CLIMB_SECOND_MOTOR_2);

    leftLiftEncoder = climbMotor1.getSensorCollection();
    rightLiftEncoder = climbMotor2.getSensorCollection();

    // different motor inversions on A and B bot
    if (Robot.checkType() == Robot.RobotType.A_BOT) {
      climbMotor1.setInverted(false);
      climbMotor2.setInverted(false);
    }

    climbMotor1.config_kP(1, 0.0001);
    climbMotor2.config_kP(1, 0.0001);

    climbMotor1.config_kI(1, 0.00005);
    climbMotor2.config_kI(1, 0.00005);

    climbMotor1.config_kD(1, 0.000005);
    climbMotor2.config_kD(1, 0.000005);

    climbMotor1.selectProfileSlot(1, 0);
    climbMotor2.selectProfileSlot(1, 0);

    // brake mode so that it doesnt fall 
    climbMotor1.setNeutralMode(NeutralMode.Brake);
    climbMotor2.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = (secondStageMotor2.getSelectedSensorPosition() / 8192) * 360 + 62.5;
  }
  
  /**
   * Lift the climb to the mid hangar position
   */
  public void liftClimb() {
    climbMotor1.set(TalonFXControlMode.Position, LIFT_MAX_POSITION);
    climbMotor2.set(TalonFXControlMode.Position, LIFT_MAX_POSITION);
  }

  /**
   * Reverse the climb back to the starting position
   */
  public void reverseClimb() {
    climbMotor1.set(TalonFXControlMode.Position, LIFT_MIN_POSITION);
    climbMotor2.set(TalonFXControlMode.Position, LIFT_MIN_POSITION);
  }

  /**
   * Stop all movement of the stage one motors
   */
  public void stopClimbLift() {
    climbMotor1.set(ControlMode.PercentOutput, 0);
    climbMotor2.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Set both of the encoders on the lift motors to 0
   */
  public void resetLiftEncoders() {
    climbMotor1.setSelectedSensorPosition(0);
    climbMotor2.setSelectedSensorPosition(0);
  }
  
  /**
   * Allows the second stage climber to rotate forward
   */
  public void stage2RotateBackwards() {
    // TODO: determine whether these angles are correct
    if (currentAngle <= (ROTATE_MAX_ANGLE + 0.5f)) {
      secondStageMotor1.set(ControlMode.PercentOutput, -0.5);
      secondStageMotor2.set(ControlMode.PercentOutput, -0.5);
    } else {
      deactivateStage2();
    }
  }

  /**
   * Allows the second stage climber to rotate forward
   */
  public void stage2RotateForwards() {
    if (currentAngle >= (ROTATE_MIN_ANGLE - 0.5f)) {
      secondStageMotor1.set(ControlMode.PercentOutput, 0.5);
      secondStageMotor2.set(ControlMode.PercentOutput, 0.5);
    } else {
      deactivateStage2();
    }
  }

  /**
   * Stop the movement of the rotational climb arms
   */
  public void deactivateStage2() {
    secondStageMotor1.set(ControlMode.PercentOutput, 0);
    secondStageMotor2.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Gets the current angle of the rotational arm
   * 
   * @return degrees
   */
  public double getCurrentAngle() {
    return currentAngle;
  }

  /** 
   * Sets the encoder of the rotational motor to 0
   */
  public void zeroRotatingArm() {
    secondStageMotor2.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Get position from the encoder of the left lift motor
   * 
   * @return encoder ticks
   */
  private double getLeftPosition() {
    return leftLiftEncoder.getIntegratedSensorPosition();
  }

  /**
   * Get position from the encoder of the right lift motor
   * 
   * @return encoder ticks
   */
  private double getRightPosition() {
    return rightLiftEncoder.getIntegratedSensorPosition();
  }

  @Override
  public void initSendable(SendableBuilder sendable) {
    sendable.setSmartDashboardType("Climbsubsystem");
    sendable.addDoubleProperty("Current Angle", this::getCurrentAngle, null);
    sendable.addDoubleProperty("Left lift encoder", this::getLeftPosition, null);
    sendable.addDoubleProperty("Right lift encoder", this::getRightPosition, null);
  }

  /**
   * Test that each motor controller is connected.
   * 
   * @return a map of the motor's name and a boolean with true if it is connected
   */
  public Map<String, Boolean> test() {
    var motors = new HashMap<String, Boolean>();

    climbMotor1.getBusVoltage();
    motors.put("Climb motor 1", climbMotor1.getLastError() == ErrorCode.OK);

    climbMotor2.getBusVoltage();
    motors.put("Climb motor 2", climbMotor2.getLastError() == ErrorCode.OK);

    secondStageMotor1.getBusVoltage();
    motors.put("Second stage motor 1", secondStageMotor1.getLastError() == ErrorCode.OK);

    secondStageMotor2.getBusVoltage();
    motors.put("Second stage motor 2", secondStageMotor2.getLastError() == ErrorCode.OK);

    // encoder check
    motors.put("Climb rotation encoder", secondStageMotor2.getSensorCollection().getPulseWidthRiseToFallUs() != 0);
    
    return motors;
  }

}
