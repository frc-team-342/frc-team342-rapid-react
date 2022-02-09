// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private WPI_TalonFX climbMotor1;
  private WPI_TalonFX climbMotor2;
  private WPI_TalonSRX secondStageMotor1;
  private WPI_TalonSRX secondStageMotor2;
  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;
  
  //Sets the second stage climb initial angle using the rotation ticks from the hex bore encoder
  private double secondStageInitialAngle = (2048 / 8192) * 360;

  private double secondStageMaximumAngle = 115.0;
  private double secondStageMinimumAngle = 62.5;
  private double currentAngle;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    climbMotor1 = new WPI_TalonFX(8);
    climbMotor2 = new WPI_TalonFX(7);
    limitSwitch1 = new DigitalInput(0);
    limitSwitch2 = new DigitalInput(1);
    secondStageMotor1 = new WPI_TalonSRX(6);
    secondStageMotor2 = new WPI_TalonSRX(5);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentAngle = (secondStageMotor1.getSelectedSensorPosition() / 8192) * 360;
  }
  
/**
 * will only run if both switches are not triggered
 */
  public void activateClimb(){

    if (!limitSwitch1.get() && !limitSwitch2.get()){
      climbMotor1.set(ControlMode.PercentOutput, 1);
      climbMotor2.set(ControlMode.PercentOutput, 1);
    }else{
      deactivateClimb();
    }

    
  }

  public void deactivateClimb(){

    climbMotor1.set(ControlMode.PercentOutput, 0);
    climbMotor2.set(ControlMode.PercentOutput, 0);
  }

  public boolean limitSwitchTriggered(){
    return(limitSwitch1.get() && limitSwitch2.get());
  }
  

  /**
   * Allows the second stage climber to rotate forward
   */
  public void stage2RotateForward()
  {
    if(currentAngle >= secondStageMinimumAngle && currentAngle <= secondStageMaximumAngle)
    {
      secondStageMotor1.set(ControlMode.PercentOutput, 0.5);
      secondStageMotor2.set(ControlMode.PercentOutput, 0.5);
    }
    else
    {
      secondStageMotor1.set(ControlMode.PercentOutput, 0);
      secondStageMotor2.set(ControlMode.PercentOutput, 0);
    }
  }


  /**
   * Allows the second stage climber to rotate forward
   */
  public void stage2RotateBackwards()
  {
    if(currentAngle >= secondStageMinimumAngle && currentAngle <= secondStageMaximumAngle)
    {
      secondStageMotor1.set(ControlMode.PercentOutput, -0.5);
      secondStageMotor2.set(ControlMode.PercentOutput, -0.5);
    }
    else
    {
      secondStageMotor1.set(ControlMode.PercentOutput, 0);
      secondStageMotor2.set(ControlMode.PercentOutput, 0);
    }
  }
  

  //Stops the second stage climber
  public void deactivateStage2()
  {
    secondStageMotor1.set(ControlMode.PercentOutput, 0);
    secondStageMotor2.set(ControlMode.PercentOutput, 0);
  }


}
