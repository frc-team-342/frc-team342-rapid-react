// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;

  private TalonFXSensorCollection leftEncoder;
  private TalonFXSensorCollection rightEncoder;
  
  //Sets the second stage climb initial angle using the rotation ticks from the hex bore encoder
  private double secondStageInitialAngle = (2048 / 8192) * 360;

  private double secondStageMaximumAngle = 115.0;
  private double secondStageMinimumAngle = 62.5;
  private double currentAngle;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor1 = new WPI_TalonFX(CLIMB_LEFT_MOTOR);
    climbMotor2 = new WPI_TalonFX(CLIMB_RIGHT_MOTOR);

    secondStageMotor1 = new WPI_TalonSRX(CLIMB_SECOND_MOTOR_1);
    secondStageMotor2 = new WPI_TalonSRX(CLIMB_SECOND_MOTOR_2);

    limitSwitch1 = new DigitalInput(LIMIT_SWITCH_1);
    limitSwitch2 = new DigitalInput(LIMIT_SWITCH_2);

    leftEncoder = climbMotor1.getSensorCollection();
    rightEncoder = climbMotor2.getSensorCollection();

    // different motor inversions on A and B bot
    if (Robot.checkType() == Robot.RobotType.A_BOT) {
      climbMotor1.setInverted(false);
      climbMotor2.setInverted(false);
    }
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
    // TODO: move this to constants
    climbMotor1.set(TalonFXControlMode.Position, 225559);
    climbMotor2.set(TalonFXControlMode.Position, 225559);
  }

  /**
   * Reverse the climb back to the starting position
   */
  public void reverseClimb() {
    climbMotor1.set(TalonFXControlMode.Position, 0);
    climbMotor2.set(TalonFXControlMode.Position, 0);
  }

  /**
   * Stop all movement of the climb
   */
  public void stopClimbLift() {
    climbMotor1.set(ControlMode.PercentOutput, 0);
    climbMotor2.set(ControlMode.PercentOutput, 0);
  }
  
  /**
   * Allows the second stage climber to rotate forward
   */
  public void stage2RotateBackwards() {
    if (currentAngle <= (secondStageMaximumAngle + 0.5f)) {
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
    if (currentAngle >= (secondStageMinimumAngle - 0.5f)) {
      secondStageMotor1.set(ControlMode.PercentOutput, 0.5);
      secondStageMotor2.set(ControlMode.PercentOutput, 0.5);
    } else {
      deactivateStage2();
    }
  }

  /** */
  public void deactivateStage2() {
    secondStageMotor1.set(ControlMode.PercentOutput, 0);
    secondStageMotor2.set(ControlMode.PercentOutput, 0);
  }

  public double getCurrentAngle() {
    return currentAngle;
  }

  public void zeroRotatingArm() {
    secondStageMotor2.setSelectedSensorPosition(0, 0, 0);
  }

  private double getLeftAngle() {
    return leftEncoder.getIntegratedSensorPosition();
  }

  private double getRightAngle() {
    return rightEncoder.getIntegratedSensorPosition();
  }

  @Override
  public void initSendable(SendableBuilder sendable) {
    sendable.setSmartDashboardType("Climbsubsystem");
    sendable.addDoubleProperty("Current Angle", this::getCurrentAngle, null);
    sendable.addDoubleProperty("Left lift encoder", this::getLeftAngle, null);
    sendable.addDoubleProperty("Right lift encoder", this::getRightAngle, null);
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
