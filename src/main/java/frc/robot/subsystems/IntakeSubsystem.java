// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX deployLeft;
  private WPI_TalonSRX deployRight;
  
  private WPI_TalonSRX rollerMotor;

  private DigitalInput limitSwitchUp;
  private DigitalInput limitSwitchDown;

  private final double intakeSpeed = 0.5;
  private final double deploySpeed = 0.2;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // capital variable names are statically imported constants
    deployLeft = new WPI_TalonSRX(DEPLOY_LEFT_MOTOR);
    deployRight = new WPI_TalonSRX(DEPLOY_RIGHT_MOTOR);

    // i dont want to change all the methods to set both motors
    deployRight.follow(deployLeft);

    rollerMotor = new WPI_TalonSRX(ROLLER_MOTOR);

    limitSwitchUp = new DigitalInput(LIMIT_SWITCH_UP);
    limitSwitchDown = new DigitalInput(LIMIT_SWITCH_DOWN);
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
      deployLeft.set(0);
    }
    else
    {
      deployLeft.set(deploySpeed);
    }
  }

  /** 
  * Retracts the intake device
  */
  public void retractIntake(){
  
    if(limitSwitchUp.get())
    {
      deployLeft.set(0);
    }
    else
    {
      deployLeft.set(deploySpeed * -1);
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

  /**
   * Stopes the intake rollers
   */
  public void stopIntake(){
    rollerMotor.set(0);
  }

  /**
   * Test that each motor controller is connected.
   * 
   * @return a map of the motor's name and a boolean with true if it is connected
   */
  public Map<String, Boolean> test() {
    var motors = new HashMap<String, Boolean>();

    deployLeft.getBusVoltage();
    motors.put("Deploy left motor", deployLeft.getLastError() == ErrorCode.OK);

    deployRight.getBusVoltage();
    motors.put("Deploy right motor", deployRight.getLastError() == ErrorCode.OK);

    rollerMotor.getBusVoltage();
    motors.put("Roller motor", rollerMotor.getLastError() == ErrorCode.OK);
    
    return motors;
  }
}
