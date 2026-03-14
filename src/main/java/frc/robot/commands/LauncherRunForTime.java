// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LauncherRunForTime extends Command {
  /** Creates a new LauncherRunForTime. */

  double power;
  double launcherRunTime;
  double setpoint;
  Launcher launcher;
  Timer launcherTimer = new Timer();

  public LauncherRunForTime(double pow, double runTime, double setpointRotationsPerSecond) {
    
    launcher = Robot.launcher;
    launcherRunTime = runTime;
    power = pow;
    setpoint = setpointRotationsPerSecond;

    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    launcherTimer.reset();
    launcherTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SimpleMotorFeedforward m_shooterFeedforward = launcher.getMotorFeedforward();
    PIDController m_shooterFeedback = launcher.getMotorFeedback();
  
    launcher.powerToFlywheelMotor(
      m_shooterFeedforward.calculate(setpoint)
      + m_shooterFeedback.calculate(launcher.getFlywheelVelocity(), setpoint));

    if (m_shooterFeedback.atSetpoint()) {
      launcher.powerToFeederAndSpindexer();
    }

    // This might get messy
    SmartDashboard.getNumber("Launcher Timer Value", launcherTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLauncherMotors();
    launcherTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return launcherTimer.hasElapsed(launcherRunTime);
  }
}
