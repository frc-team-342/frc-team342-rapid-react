// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

  private WPI_TalonFX leftClimbLift;
  private WPI_TalonFX rightClimbLift;

  private TalonFXSensorCollection leftClimbLiftEncoder;
  private TalonFXSensorCollection rightClimbLiftEncoder;

  private WPI_TalonSRX leadClimbRotate;
  private WPI_TalonSRX followClimbRotate;

  // Used for locking the climber controls on the operator control before climb time
  private boolean climbMode;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    leftClimbLift = new WPI_TalonFX(LEFT_LIFT_MOTOR);
    rightClimbLift = new WPI_TalonFX(RIGHT_LIFT_MOTOR);

    leftClimbLiftEncoder = leftClimbLift.getSensorCollection();
    rightClimbLiftEncoder = rightClimbLift.getSensorCollection();

    leadClimbRotate = new WPI_TalonSRX(LEAD_ROTATE_MOTOR);
    followClimbRotate = new WPI_TalonSRX(FOLLOW_ROTATE_MOTOR);

    followClimbRotate.follow(leadClimbRotate);

    // brake mode so that it does not fall when on the bars
    leftClimbLift.setNeutralMode(NeutralMode.Brake);
    rightClimbLift.setNeutralMode(NeutralMode.Brake);

    // locks or activates the climbers
    climbMode = false;
  }
  
  @Override
  public void periodic() {}

  /**
   * Moves the lift mechanism of the climber. <br>
   * Will not move outside of a window of encoder values for minimum and maximum positions.
   * 
   * @param speed the speed to move the climber at, [-1, 1]
   */
  public void liftClimb(double speed) {
    double leftPosition = leftClimbLiftEncoder.getIntegratedSensorPosition();
    double rightPosition = rightClimbLiftEncoder.getIntegratedSensorPosition();

    // only run if climb has been activated
    if (climbMode) {
      // above minimum position on each encoder since arms are not mechanically linked
      boolean leftAboveMin = (leftPosition >= LIFT_MIN_POSITION);
      boolean leftBelowMax = (leftPosition <= LIFT_MAX_POSITION);

      boolean rightAboveMin = (rightPosition >= LIFT_MIN_POSITION);
      boolean rightBelowMax = (rightPosition <= LIFT_MAX_POSITION);

      // only runs outside of bounds if it is moving back towards bounds
      if ((leftAboveMin || speed > 0) && (leftBelowMax || speed < 0)) {
        leftClimbLift.set(speed);
      } else {
        // otherwise do not move
        leftClimbLift.set(0);
      }

      if ((rightAboveMin || speed > 0) && (rightBelowMax || speed < 0)) {
        rightClimbLift.set(speed);
      } else {
        rightClimbLift.set(0);
      }
    } else {
      // do not run motors if climb mode is not enabled
      leftClimbLift.set(0);
      rightClimbLift.set(0);
    }
  }

  /**
   * Moves the rotational mechanism of the climber. 
   * Will not move outside of a window of angles for minimum and maximum position.
   *  
   * @param speed the speed to rotate at, [-1, 1]
   */
  public void rotateClimb(double speed) {
    double position = leadClimbRotate.getSelectedSensorPosition();

    if (climbMode) {
      boolean withinMinAngle = encoderTicksToDegrees(position) > ROTATE_MIN_ANGLE;
      boolean withinMaxAngle = encoderTicksToDegrees(position) > ROTATE_MAX_ANGLE;

      // only run if within bounds or moving back towards within bounds
      if ((withinMinAngle || speed > 0) && (withinMaxAngle || speed < 0)) {
        leadClimbRotate.set(speed);
      }
    } else {
      // do not run if climb mode is not enabled
      leadClimbRotate.set(0);
    }
  }

  /**
   * Converts encoder ticks to degrees of motor rotation
   * 
   * @param ticks encoder ticks
   * @return degrees, continuous past 360
   */
  private double encoderTicksToDegrees(double ticks) {
    // 8192 encoder ticks in a rotation, 360 degrees in a rotation
    return (ticks / ROTATE_ENCODER_TICKS_PER_ROT) * 360;
  }

  /**
   * Stop the movement of the climb lift motors.
   */
  public void stopClimbLift() {
    leftClimbLift.set(0);
    rightClimbLift.set(0);
  }

  /**
   * Stop the movement of the climb rotate motors.
   */
  public void stopClimbRotate() {
    leadClimbRotate.set(0);
  }

  @Override
  public void initSendable(SendableBuilder sendable) {
    sendable.setSmartDashboardType("ClimbSubsystem");
  }

  /**
   * Test that each motor controller is connected.
   * 
   * @return a map of the motor's name and a boolean with true if it is connected
   */
  public Map<String, Boolean> test() {
    var motors = new HashMap<String, Boolean>();

    leftClimbLift.getBusVoltage();
    motors.put("Climb motor 1", leftClimbLift.getLastError() == ErrorCode.OK);

    rightClimbLift.getBusVoltage();
    motors.put("Climb motor 2", rightClimbLift.getLastError() == ErrorCode.OK);

    leadClimbRotate.getBusVoltage();
    motors.put("Second stage motor 1", leadClimbRotate.getLastError() == ErrorCode.OK);

    followClimbRotate.getBusVoltage();
    motors.put("Second stage motor 2", followClimbRotate.getLastError() == ErrorCode.OK);

    // encoder check
    motors.put("Climb rotation encoder", followClimbRotate.getSensorCollection().getPulseWidthRiseToFallUs() != 0);
    
    return motors;
  }
}
