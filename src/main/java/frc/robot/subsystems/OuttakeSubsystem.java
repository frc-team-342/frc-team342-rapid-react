// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.OuttakeConstants.*;

public class OuttakeSubsystem extends SubsystemBase {

  private WPI_TalonFX shootMotor1;
  private WPI_TalonFX shootMotor2;
  private WPI_TalonSRX feederMotor;

  /** Encoder from motor 1 */
  private TalonFXSensorCollection encoder1;
  private TalonFXSensorCollection encoder2;

  /** Current limiting for both motors. */
  private SupplyCurrentLimitConfiguration currentLimitConfiguration;

  /** The current setpoint */
  private double setpoint = 0;

  /** The current motor velocity, as measured by encoders */
  private double velocity = 0;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    shootMotor1 = new WPI_TalonFX(SHOOT_MOTOR_1);
    shootMotor2 = new WPI_TalonFX(SHOOT_MOTOR_2);
    feederMotor = new WPI_TalonSRX(FEEDER_MOTOR);

    if (Robot.checkType() == Robot.RobotType.A_BOT) {
      shootMotor1.setInverted(true);
      shootMotor2.setInverted(false);
      feederMotor.setInverted(false);
    } else {
      shootMotor1.setInverted(false);
      shootMotor2.setInverted(true);
      feederMotor.setInverted(true);
    }

    encoder1 = shootMotor1.getSensorCollection();
    encoder2 = shootMotor2.getSensorCollection();

    // P and D are statically imported constants
    shootMotor1.config_kP(1, P);
    shootMotor2.config_kP(1, P);

    shootMotor1.config_kI(1, I);
    shootMotor2.config_kI(1, I);

    shootMotor1.config_kD(1, D);
    shootMotor2.config_kD(1, D);

    shootMotor1.selectProfileSlot(1, 0);
    shootMotor2.selectProfileSlot(1, 0);

    currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
      true, // enabled or not
      CURRENT_LIMIT, // current to limit to after 
      CURRENT_THRESHOLD, // current to start limiting at
      TIMEOUT // time in seconds before current limit takes effect
    );

    shootMotor1.configSupplyCurrentLimit(currentLimitConfiguration);
    shootMotor2.configSupplyCurrentLimit(currentLimitConfiguration);
  }

   /**
    * Sets speed of motors in order to shoot in low goal
    */
  public void shootLow(){
    setpoint = LOW_RPM;
  }

  /**
   * Sets speed of motors in order to shoot in high goal
   */
  public void shootHigh(){
    setpoint = HIGH_RPM;
  }

  /**
   * Sets speed of motors to 0 to stop motor's shooting
   */
  public void stopShooter(){
    setpoint = 0;
  }

  /**
   * Check if the shooter is at its setpoint, within the error margin set by setpointError.
   * 
   * @return true if it is within the margin of error
   */
  public boolean upToSpeed() {
    // Units are in encoder units per 100 ms right now
    velocity = encoder1.getIntegratedSensorVelocity();

    // converting from encoder ticks / 100 ms to rotations per minutes
    // ((velocity * 10 ms) * 60 s) / 2048 ticks
    double rpm = (velocity * 60 * 10) / 2048;

    // check if rpm is within tolerance
    if (setpoint != 0) {  
      return rpm >= (setpoint - RPM_ERROR) && rpm <= (setpoint + RPM_ERROR);
    }
    else {
      return false;
    }
  }

  /**
   * @return the current setpoint of the shooter
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Setpoint adjusted to units used for CTRE motors
   * 
   * @return encoder ticks per 100 ms
   */
  public double getAdjustedSetpoint() {
    // 2048 is encoder ticks per rotation
    // 60 * 10 is minutes to seconds and seconds to 100 ms
    return (setpoint * 2048) / 600;
  }

  /**
   * Distance between both encoders
   * 
   * @return encoder ticks
   */
  private double getEncoderDelta() {
    return encoder2.getIntegratedSensorPosition() + encoder1.getIntegratedSensorPosition();
  }

  /**
   * Set the position of both encoders to 0
   */
  public void resetEncoders() {
    encoder1.setIntegratedSensorPosition(0, 10);
    encoder2.setIntegratedSensorPosition(0, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shootMotor1.set(TalonFXControlMode.Velocity, getAdjustedSetpoint());
    shootMotor2.set(TalonFXControlMode.Velocity, getAdjustedSetpoint());

    // the velocity variable is updated in period by the upToSpeed() method

    // Open loop control is used on feed motors
    if (upToSpeed() && setpoint != 0) {
      // Only feed if the shooter is ready
      feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    } else {
      feederMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * Current velocity of the shooter motors
   * 
   * @return RPM
   */
  private double getVelocity() {
    return (velocity * 600) / 2048;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("OuttakeSubsystem");
    builder.addBooleanProperty("Up to speed", this::upToSpeed, null);
    builder.addDoubleProperty("Setpoint", this::getSetpoint, null);
    builder.addDoubleProperty("Adjusted setpoint", this::getAdjustedSetpoint, null);

    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Encoder delta", this::getEncoderDelta, null);
  }

  /**
   * Test that each motor controller is connected.
   * 
   * @return a map of the motor's name and a boolean with true if it is connected
   */
  public Map<String, Boolean> test() {
    Map<String, Boolean> motors = new HashMap<>();

    // Name of motor controller and whether it is physically connected
    shootMotor1.getBusVoltage();
    motors.put("Shooter motor 1", shootMotor1.getLastError() == ErrorCode.OK);
    
    shootMotor2.getBusVoltage();
    motors.put("Shooter motor 2", shootMotor2.getLastError() == ErrorCode.OK);

    feederMotor.getBusVoltage();
    motors.put("Feeder motor", feederMotor.getLastError() == ErrorCode.OK);

    return motors;
  }
}
