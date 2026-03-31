// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ExtendIntaketoPos;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakePower;
import frc.robot.commands.IntakeRunForTime;
import frc.robot.commands.LauncherRunForTime;
import frc.robot.commands.RetractInake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    private SendableChooser<Command> autoChooser;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final FieldCentricFacingAngle rotateToAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController gunner = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double rotateToHubAngle = 0;

    public RobotContainer() {

        configureBindings();

        NamedCommands.registerCommand("Stop Launcher", Robot.launcher.stopLauncherCommand());
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("Shoot", new LauncherRunForTime(Constants.LauncherConstants.DESIRED_RPS, 5.0));
        NamedCommands.registerCommand("Shoot longer", new LauncherRunForTime(Constants.LauncherConstants.DESIRED_RPS, 7.0));
        NamedCommands.registerCommand("Extend Intake", new ExtendIntaketoPos(Constants.PowerConstants.INTAKE_POSITION_OUT_POWER, Constants.IntakeConstants.INTAKE_OUT_POSITION));
        NamedCommands.registerCommand("Run Intake", new IntakeRunForTime(Constants.PowerConstants.INTAKE_RUN_POWER, 4.0));
        NamedCommands.registerCommand("Retract Intake", new RetractInake(Constants.PowerConstants.INTAKE_POSITION_IN_POWER));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodrically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // TODO: Allegedly rotates the robot to Hub (needs testing and tweaking perhaps sequential command groud with X stance)

        driver.rightBumper().onTrue(Commands.runOnce(() -> setDesiredAngle(), drivetrain));
        driver.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> rotateToAngle
                .withTargetDirection(Rotation2d.fromDegrees(rotateToHubAngle))
                .withVelocityX(0)
                .withVelocityY(0)
            )
        );

        //gunner controls
        
        gunner.rightBumper().whileTrue(Robot.launcher.shootCommand(Constants.LauncherConstants.DESIRED_RPS));
        //gunner.rightBumper().whileTrue(Robot.launcher.shootCommand2());

        // gunner.rightBumper().whileTrue(new RunFeeder());
        gunner.y().whileTrue(Robot.launcher.run(Robot.launcher::powerToFeederAndSpindexer));
                
        gunner.povDown().whileTrue(new IntakeIn(Constants.PowerConstants.INTAKE_POSITION_IN_POWER));
        gunner.povUp().whileTrue(new IntakeOut(Constants.PowerConstants.INTAKE_POSITION_OUT_POWER));
        gunner.leftBumper().whileTrue(new IntakePower(Constants.PowerConstants.INTAKE_RUN_POWER));
        gunner.leftTrigger().whileTrue(new IntakePower(Constants.PowerConstants.INTAKE_REVERSE_RUN_POWER));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        auto.addRequirements(drivetrain);
        return auto;
    }

    public CommandSwerveDrivetrain getSwerve() {
        return drivetrain;
    }

    public void setDesiredAngle() {
        rotateToHubAngle = drivetrain.getAngleToHub();
    }
}
