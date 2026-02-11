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


public class IntakeOutIn extends SubsystemBase {


  public final DigitalInput OutsideLimitSwitch;
  public final DigitalInput InsideLimitSwitch;

 private final SparkMax IntakePositionMotor;

    /** Creates a new IntakeOutIn. */
  public IntakeOutIn() {


 IntakePositionMotor = new SparkMax(Constants.MotorIDs.INTAKE_POSITION_MOTOR_ID, MotorType.kBrushless);
 InsideLimitSwitch = new DigitalInput(Constants.MotorIDs.INTAKE_INSIDE_LIMIT_SWITCH_ID);
 OutsideLimitSwitch = new DigitalInput(Constants.MotorIDs.INTAKE_OUTSIDE_LIMIT_SWITCH_ID);



  }

public void PowerToIntakeOut(double percentPower){
  
 if (OutsideLimitSwitch.get() || (!OutsideLimitSwitch.get() && percentPower >= 0)) {
            IntakePositionMotor.set(percentPower);
        } 
        
        else 
        {
            IntakePositionMotor.set(0);
        }

}

public void PowerToIntakeIn(double percentPower){
  
 if (InsideLimitSwitch.get() || (!OutsideLimitSwitch.get() && percentPower >= 0)) {
            IntakePositionMotor.set(percentPower);
        } 
        
        else 
        {
            IntakePositionMotor.set(0);
        }

}

 public double getIntakeMotorPosition() {

    return IntakePositionMotor.getEncoder().getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
