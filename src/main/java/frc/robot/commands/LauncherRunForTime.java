// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LauncherRunForTime extends Command {

  private final double desiredRPS;
  private final double runTime;
  private final Timer launcherTimer = new Timer();
  private final Launcher launcher;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.KS_VOLTS, LauncherConstants.KV_VOLTS, LauncherConstants.KA_VOLTS);
  private final PIDController m_shooterFeedback = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);

  /** Creates a new IntakeRunForTime. */
  public LauncherRunForTime(double desiredRotationsPerSecond, double runtime) {
    // Use addRequirements() here to declare subsystem dependencies.
    launcher = Robot.launcher;
    addRequirements(launcher);

    desiredRPS = desiredRotationsPerSecond;
    runTime = runtime;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherTimer.reset();
    launcherTimer.start();
    System.out.println("power: " + desiredRPS);
    System.out.println("RunTime: " + runTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Launcher RPS: " + launcher.getFlywheelVelocity());
    SmartDashboard.putNumber("Desired Launcher RPS", desiredRPS);
    SmartDashboard.putNumber("Launcher RPS2", launcher.getFlywheelVelocity());
    launcher.shootFuel(desiredRPS);

    if (launcher.getFlywheelVelocity() > desiredRPS - LauncherConstants.KSHOOTER_TOLERANCE_RPS) {
      launcher.powerToFeederAndSpindexer();
    }
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
    return launcherTimer.hasElapsed(runTime);
  }
}
