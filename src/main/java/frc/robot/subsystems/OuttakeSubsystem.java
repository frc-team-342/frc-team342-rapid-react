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

  }

  public void shootLow(){

    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    shootMotor1.set(ControlMode.PercentOutput, 0.4);
    shootMotor2.set(ControlMode.PercentOutput, 0.4);


  }

  public void shootHigh(){

    feederMotor.set(ControlMode.PercentOutput, LOAD_SPEED);
    shootMotor1.set(ControlMode.PercentOutput, 0.8);
    shootMotor2.set(ControlMode.PercentOutput, 0.8);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
