// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    shootMotor2.follow(shootMotor1);

    // TODO: move to constants
    shootMotor1.config_kP(1, 0.0);
    shootMotor2.config_kP(1, 0.0);

    shootMotor1.config_kD(1, 0.0);
    shootMotor2.config_kD(1, 0.0);

    shootMotor1.selectProfileSlot(1, 0);
    shootMotor2.selectProfileSlot(1, 0);
  }

   /**
    * Sets speed of motors in order to shoot in low goal
    */
  public void shootLow(){
    // Open loop control is used on feed motors
    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);

    // Units is encoder ticks / 100 ms
    shootMotor1.set(ControlMode.Velocity, 0);
  }

  /**
   * Sets speed of motors in order to shoot in high goal
   */
  public void shootHigh(){
    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    shootMotor1.set(ControlMode.PercentOutput, 0.8);
  }

  /**
   * Sets speed of motors to 0 to stop motor's shooting
   */
  public void stopShooter(){
    feederMotor.set(0);
    shootMotor1.set(0);
  }

  /**
   * 
   * 
   * @return
   */
  public boolean upToSpeed() {
    // TODO: fix it bruh
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
