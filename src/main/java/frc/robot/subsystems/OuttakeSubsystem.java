// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OuttakeConstants.*;

public class OuttakeSubsystem extends SubsystemBase {

  private WPI_TalonFX shootMotor1;
  private WPI_TalonFX shootMotor2;
  private WPI_TalonSRX feederMotor;

  /** The current setpoint */
  private double setpoint = 0;

  /** RPM within the setpoint to be counted as up to speed */
  private double setpointError = 15;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {
    shootMotor1 = new WPI_TalonFX(SHOOT_MOTOR_1);
    shootMotor2 = new WPI_TalonFX(SHOOT_MOTOR_2);
    feederMotor = new WPI_TalonSRX(FEEDER_MOTOR);

    shootMotor2.setInverted(true);

    feederMotor.setInverted(true);

    // P and D are statically imported constants
    shootMotor1.config_kP(1, P);
    shootMotor2.config_kP(1, P);

    shootMotor1.config_kD(1, D);
    shootMotor2.config_kD(1, D);

    shootMotor1.selectProfileSlot(1, 0);
    shootMotor2.selectProfileSlot(1, 0);
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
    double velocity = shootMotor1.getSensorCollection().getIntegratedSensorVelocity();

    // converting from encoder ticks / 100 ms to rotations per minutes
    // ((velocity * 10 ms) * 60 s) / 2048 ticks
    double rpm = (velocity * 60 * 10) / 2048;

    // check if rpm is within tolerance
    if (setpoint != 0) {  
      return rpm >= (setpoint - setpointError) && rpm <= (setpoint + setpointError);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   shootMotor1.set(TalonFXControlMode.Velocity, getAdjustedSetpoint());
   shootMotor2.set(TalonFXControlMode.Velocity, getAdjustedSetpoint());

    // Open loop control is used on feed motors
    if (upToSpeed() && setpoint != 0) {
      // Only feed if the shooter is ready
      feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    } else {
      feederMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("OuttakeSubsystem");
    builder.addBooleanProperty("Up to speed", this::upToSpeed, null);
    builder.addDoubleProperty("Setpoint", this::getSetpoint, null);
    builder.addDoubleProperty("Adjusted setpoint", this::getAdjustedSetpoint, null);
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
