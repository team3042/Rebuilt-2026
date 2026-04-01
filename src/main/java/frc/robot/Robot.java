// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public final RobotContainer m_robotContainer;

    public static Launcher launcher = new Launcher();
    public static Intake intake = new Intake();

    // This is in case we want to use the target for aiming at a hub target using the back camera.
    public static PhotonTrackedTarget bestBackTarget;
    public static int latestFiducialId = 0;
    public static double distanceToTarget = 0;

    UsbCamera camera1;
    UsbCamera camera2;

    private Vision vision;

    /* log and replay timestamp and joystick data */
    @Override   public void robotInit() {
        vision = new Vision(m_robotContainer.drivetrain::addVisionMeasurement);

        camera1 = CameraServer.startAutomaticCapture(0);
        camera1.setResolution(320,240);
        camera1.setFPS(15);
        camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        camera2 = CameraServer.startAutomaticCapture(1);
        camera2.setResolution(320,240);
        camera2.setFPS(15);
        camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen); }

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        PortForwarder.add(5800, "photonvision.local", 5800);
        DataLogManager.start();
        m_robotContainer.getSwerve().resetPose(new Pose2d(3.6, 4, new Rotation2d()));

    }

    @Override
    public void robotPeriodic() {
        
        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run(); 

        SmartDashboard.putNumber("Intake Encoder Counts", intake.getIntakeMotorPosition());
        SmartDashboard.putBoolean("Intake Inside Limit Switch", intake.insideLimitSwitch.get());
        SmartDashboard.putNumber("Flywheel Speed", launcher.getFlywheelVelocity());
        SmartDashboard.putNumber("True FW Velocity", launcher.getTrueFlywheelVelocity());
        SmartDashboard.putNumber("Spindexer Velocity", launcher.getSpindexerMotorVelocity());
        SmartDashboard.putNumber("Feeder Velocity", launcher.getFeederMotorVelocity());
        Pose2d robotPose = m_robotContainer.getSwerve().getState().Pose;
        SmartDashboard.putString("Robot Pos", robotPose.getX() + ", " + robotPose.getY());
        SmartDashboard.putNumber("Robot speed (X)", m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond);

        vision.periodic();

        if (bestBackTarget != null) {
            distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(0.4, 1.124, Units.degreesToRadians(0), Units.degreesToRadians(bestBackTarget.getPitch()));
            latestFiducialId = bestBackTarget.fiducialId;
        }
        SmartDashboard.putNumber("AprilTag #", latestFiducialId);
        SmartDashboard.putNumber("Distance to target", distanceToTarget);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            System.out.println("Auto selected: " + m_autonomousCommand.getName());
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
