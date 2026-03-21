// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRunForTime extends Command {

  double power;
  double intakeRunTime;
  Timer intakeTimer = new Timer();
  Intake intake;


  /** Creates a new IntakeRunForTime. */
  public IntakeRunForTime(double powerlocal, double runtime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
    power = powerlocal;
    intakeRunTime = runtime;
    intake = Robot.intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();
    System.out.println("power: " + power);
    System.out.println("RunTime: " + intakeRunTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.powerToIntakeRun(power);
    System.out.println("setting intake POWER");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRunMotor();
    intakeTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeTimer.hasElapsed(intakeRunTime);
  }
}
