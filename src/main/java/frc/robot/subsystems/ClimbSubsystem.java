// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  
  //Converts the rotation ticks from the stage 2 encoders to degrees
  private double secondStageInitialAngle = (2048 / 8192) * 360;
  private double secondStageMaximumPositiveAngle;
  private double secondStageMaximumNegativeAngle;


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
   * Checks if the second stage climber is the appropriate angle range
   * Moves the climber to the appropriate location depending on the determined range
   * If outside the determined range, stops the second stage climber
   */
  public void activateStage2()
  {
    if((secondStageMotor1.getSelectedSensorPosition() <= secondStageInitialAngle + 5 || 
    secondStageMotor1.getSelectedSensorPosition() >= secondStageInitialAngle - 5) 
    && (secondStageMotor1.getSelectedSensorPosition() <= secondStageMaximumPositiveAngle + 5 
    || secondStageMotor1.getSelectedSensorPosition() >= secondStageMaximumPositiveAngle - 5))
    {
      secondStageMotor1.set(ControlMode.PercentOutput, 1);
      secondStageMotor2.set(ControlMode.PercentOutput, 1);
    }

    else if((secondStageMotor1.getSelectedSensorPosition() >= secondStageInitialAngle + 5 ||
    secondStageMotor1.getSelectedSensorPosition() >= secondStageInitialAngle - 5) && 
    (secondStageMotor1.getSelectedSensorPosition() <= secondStageMaximumNegativeAngle + 5 ||
    secondStageMotor1.getSelectedSensorPosition() >= secondStageMaximumNegativeAngle - 5))
    {
      secondStageMotor1.set(ControlMode.PercentOutput, -1);
      secondStageMotor2.set(ControlMode.PercentOutput, -1);
    }

    else
    {
      deactivateStage2();
    }
  }

  //Stops the second stage climber
  public void deactivateStage2()
  {
    secondStageMotor1.set(ControlMode.PercentOutput, 0);
    secondStageMotor2.set(ControlMode.PercentOutput, 0);
  }


}
