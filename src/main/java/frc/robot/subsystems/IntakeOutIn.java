// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;


public class IntakeOutIn extends SubsystemBase {


  public final DigitalInput outsideLimitSwitch;
  public final DigitalInput insideLimitSwitch;

 private final SparkMax intakePositionMotor;

    /** Creates a new IntakeOutIn. */
  public IntakeOutIn() {


 intakePositionMotor = new SparkMax(Constants.MotorIDs.INTAKE_POSITION_MOTOR_ID, MotorType.kBrushless);
 insideLimitSwitch = new DigitalInput(Constants.DigitalIO.INTAKE_INSIDE_LIMIT_SWITCH_ID);
 outsideLimitSwitch = new DigitalInput(Constants.DigitalIO.INTAKE_OUTSIDE_LIMIT_SWITCH_ID);



  }

public void powerToIntakeOut(double percentPower){
  
 if (outsideLimitSwitch.get() || (!insideLimitSwitch.get() && percentPower >= 0)) {
            intakePositionMotor.set(percentPower);
        } 
        
        else 
        {
            intakePositionMotor.set(0);
        }

}

public void powerToIntakeIn(double percentPower){
  
 if (insideLimitSwitch.get() || (!outsideLimitSwitch.get() && percentPower <= 0)) {
            intakePositionMotor.set(percentPower);
        } 
        
        else 
        {
            intakePositionMotor.set(0);
        }

}

 public double getIntakeMotorPosition() {

    return intakePositionMotor.getEncoder().getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
