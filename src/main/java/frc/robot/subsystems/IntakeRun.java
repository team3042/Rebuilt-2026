// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRun extends SubsystemBase {

  private final SparkMax intakeMotor;

  /** Creates a new IntakeRun. */
  public IntakeRun() {

    intakeMotor = new SparkMax(Constants.MotorIDs.INTAKE_RUN_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

  }

  public void intakePower(double percentPower) {
    intakeMotor.set(percentPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
