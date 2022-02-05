// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  private WPI_TalonFX shootMotor1;
  private WPI_TalonFX shootMotor2;
  private WPI_TalonSRX feederMotor;
  private final double LOAD_SPEED = 0.8;

  /** Creates a new OuttakeSubsystem. */
  public OuttakeSubsystem() {

    shootMotor1 = new WPI_TalonFX(5);
    shootMotor2 = new WPI_TalonFX(6);
    feederMotor = new WPI_TalonSRX(7);

  }

   /**
    * Sets speed of motors in order to shoot in low goal
    */
  public void shootLow(){

    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    shootMotor1.set(ControlMode.PercentOutput, 0.4);
    shootMotor2.set(ControlMode.PercentOutput, 0.4);
  }

  /**
    * Sets speed of motors in order to shoot in high goal
    */
  public void shootHigh(){

    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    shootMotor1.set(ControlMode.PercentOutput, 0.8);
    shootMotor2.set(ControlMode.PercentOutput, 0.8);

  }

  /**
   * Sets speed of motors to 0 to stop motor's shooting
   */
  public void stopShooter(){

    feederMotor.set(0);
    shootMotor1.set(0);
    shootMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
