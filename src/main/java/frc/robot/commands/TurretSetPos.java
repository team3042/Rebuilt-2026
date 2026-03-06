// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretSetPos extends Command {
  /** Creates a new TurretSetPos. */

  Turret turret;
  double encoderCountsGoal;
  Pose2d robotPose;
  Translation2d hubCenter;
  Rotation2d robotRotation;

  public TurretSetPos(CommandSwerveDrivetrain drivetrain) {

    addRequirements(Robot.turret);
    
    robotPose = drivetrain.getState().Pose;
    robotRotation = drivetrain.getState().RawHeading;

    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      hubCenter = Constants.FieldConstants.Hub.redTopCenterPoint.toTranslation2d();
    } else if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      hubCenter = Constants.FieldConstants.Hub.blueTopCenterPoint.toTranslation2d();
    }
  }

  //finds the angle the turret needs to be pointed in degrees
  //TODO: experiment what way X and Y truly is. like is positive Y negative or positive?
  private double findTurretAngle() {

      double robotX = robotPose.getX();
      double robotY = robotPose.getY();

      double hubX = hubCenter.getX();
      double hubY = hubCenter.getY();

      double distX = hubX - robotX;
      double distY = hubY - robotY;

      double robotAngle = robotRotation.getDegrees();

      double turretDegree = Math.toDegrees(Math.atan(distY/distX)) - robotAngle;

      return (distX > 0) ? turretDegree : turretDegree + 180;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double goalAngle = findTurretAngle();
    encoderCountsGoal = Turret.angleToEncoderCount(goalAngle);
    SmartDashboard.putNumber("Turret Angle Goal", goalAngle);

    double inputPower = 
    //if turret ISN'T in the stop turret rotation threshold
      (Math.abs(turret.getEncoderCounts() - encoderCountsGoal) > Constants.TurretConstants.TURRET_STOP_ENCODER_COUNT_TOLERANCE) ? 
      // if the turret IS in the slow turret rotation threshold
        (Math.abs(turret.getEncoderCounts() - encoderCountsGoal) < Constants.TurretConstants.TURRET_SLOW_ENCODER_COUNT_TOLERANCE) ?
        // if the turret needs to rotate clockwise to meet the goal
          (turret.getEncoderCounts() < encoderCountsGoal) ? Constants.TurretConstants.TURRET_ROTATE_CLOCKWISE_AUTO_SLOW_POWER 
            : Constants.TurretConstants.TURRET_ROTATE_COUNTERCLOCKWISE_AUTO_SLOW_POWER 
          : (turret.getEncoderCounts() < encoderCountsGoal) ? Constants.TurretConstants.TURRET_ROTATE_CLOCKWISE_AUTO_FAST_POWER 
        : Constants.TurretConstants.TURRET_ROTATE_COUNTERCLOCKWISE_AUTO_FAST_POWER 
      : 0;

    turret.powerToTurret(inputPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.powerToTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(turret.getEncoderCounts() - encoderCountsGoal) < Constants.TurretConstants.TURRET_STOP_ENCODER_COUNT_TOLERANCE;
  }
}
