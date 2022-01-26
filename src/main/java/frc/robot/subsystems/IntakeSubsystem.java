// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {
WPI_TalonSRX intakeDeployMotor;
WPI_TalonSRX intakeRollerMotor;

private DigitalInput intakeLimitSwitchU;
private DigitalInput intakeLimitSwitchD;

private final double intakeSpeed = 0.5;
private final double deploySpeed = 0.2;




  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeDeployMotor = new WPI_TalonSRX(0);
    intakeRollerMotor = new WPI_TalonSRX(1);

    intakeLimitSwitchU = new DigitalInput(0);
    intakeLimitSwitchD = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void deployIntake()
  {
    if(intakeLimitSwitchD.get())
    {
      intakeDeployMotor.set(0);
    }
    else
    {
    intakeDeployMotor.set(deploySpeed);
    }
  }

  public void retractIntake()
  {
    if(intakeLimitSwitchU.get())
    {
      intakeDeployMotor.set(0);
    }
    else
    {
      intakeDeployMotor.set(deploySpeed * -1);
    }
  }

  public void activateIntake()
  {
    intakeRollerMotor.set(intakeSpeed);
  }

  public void reverseIntake()
  {
    intakeRollerMotor.set(intakeSpeed * -1);
  }
}
