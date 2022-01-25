// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private TalonSRX climbMotor1;
  private TalonSRX climbMotor2;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    TalonSRX climbMotor1 = new TalonSRX(8);
    TalonSRX climbMotor2 = new TalonSRX(8);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void activateClimb(){

    climbMotor1.set(ControlMode.PercentOutput, 1);
    climbMotor2.set(ControlMode.PercentOutput, 1);
  }

  public void deactivateClimb(){

    climbMotor1.set(ControlMode.PercentOutput, 0);
    climbMotor2.set(ControlMode.PercentOutput, 0);
  }

}
