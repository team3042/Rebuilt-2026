// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeOutIn extends SubsystemBase {


  public final DigitalInput outsideLimitSwitch;
  public final DigitalInput insideLimitSwitch;

  private final SparkMax intakePositionMotor;
  private final SparkMax IntakeRunMotor;

    /** Creates a new IntakeOutIn. */
  public IntakeOutIn() {

    intakePositionMotor = new SparkMax(Constants.MotorIDs.INTAKE_POSITION_MOTOR_ID, MotorType.kBrushless);
    IntakeRunMotor = new SparkMax(Constants.MotorIDs.INTAKE_RUN_MOTOR_ID, MotorType.kBrushless);

    InsideLimitSwitch = new DigitalInput(Constants.DigitalIO.INTAKE_INSIDE_LIMIT_SWITCH_ID);
    OutsideLimitSwitch = new DigitalInput(Constants.DigitalIO.INTAKE_OUTSIDE_LIMIT_SWITCH_ID);

  }

  //method that gets Intake into position
  public void PowerToIntakeOut(double percentPower){
    
      percentPower = Math.min(percentPower, 1);
      percentPower = Math.max(percentPower, -1);

      if (!OutsideLimitSwitch.get() || (!InsideLimitSwitch.get() && percentPower >= 0)) {
          intakePositionMotor.set(percentPower);
      } else {
          intakePositionMotor.set(0);
      }

  }

  //method that gets Intake back in the robot
  public void PowerToIntakeIn(double percentPower){
    
      percentPower = Math.min(percentPower, 1);
      percentPower = Math.max(percentPower, -1);

      if (InsideLimitSwitch.get() || (!OutsideLimitSwitch.get() && percentPower >= 0)) {
          intakePositionMotor.set(percentPower);
      } else {
          intakePositionMotor.set(0);
      }
  }

  public void PowerToIntakeRun(double percentPower) {

    percentPower = Math.min(percentPower, 1);
    percentPower = Math.max(percentPower, -1);

    IntakeRunMotor.set(percentPower);
  }

  public void stopPositionMotor() {
    intakePositionMotor.set(0);
  }

  public void stopRunMotor() {
    IntakeRunMotor.set(0);
  }

  public double getIntakeMotorPosition() {

      return intakePositionMotor.getEncoder().getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
