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

    leadClimbRotate = new WPI_TalonSRX(LEAD_ROTATE_MOTOR);
    followClimbRotate = new WPI_TalonSRX(FOLLOW_ROTATE_MOTOR);

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
      // above minimum position and below maximum position
      boolean leftInPositionWindow = (leftPosition >= LIFT_MIN_POSITION) && (leftPosition <= LIFT_MAX_POSITION);
      boolean rightInPositionWindow = (rightPosition >= LIFT_MIN_POSITION) && (rightPosition <= LIFT_MAX_POSITION);

      // only run if motors are within acceptable positions
      if (leftInPositionWindow) {
        leftClimbLift.set(speed);
      }

      if (rightInPositionWindow) {
        rightClimbLift.set(speed);
      }
    }
  }

  /**
   * Stop the movement of the climb lift motors.
   */
  public void stopClimbLift() {
    leftClimbLift.set(0);
    rightClimbLift.set(0);
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
