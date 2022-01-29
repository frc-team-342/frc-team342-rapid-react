// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX deployMotor;
  private WPI_TalonSRX rollerMotor;

  private DigitalInput limitSwitchUp;
  private DigitalInput limitSwitchDown;

  private final double intakeSpeed = 0.5;
  private final double deploySpeed = 0.2;




  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    deployMotor = new WPI_TalonSRX(0);
    rollerMotor = new WPI_TalonSRX(1);


    limitSwitchUp = new DigitalInput(0);
    limitSwitchDown = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** 
  * Deploys the intake device for picking up cargo
  */
  public void deployIntake()
  {
    if(limitSwitchDown.get())
    {
      deployMotor.set(0);
    }
    else
    {
      deployMotor.set(deploySpeed);
    }
  }

  /** 
  * Retracts the intake device
  */
  public void retractIntake(){
  
    if(limitSwitchUp.get())
    {
      deployMotor.set(0);
    }
    else
    {
      deployMotor.set(deploySpeed * -1);
    }
  }

  /**
   * Activates the intake rollers to collect cargo
   */
  public void intakeCargo()
  {
    rollerMotor.set(intakeSpeed);
  }

  /**
   * Reverses the intake system to remove jammed cargo
   */
  public void reverseIntakeCargo()
  {
    rollerMotor.set(intakeSpeed * -1);
  }
}
