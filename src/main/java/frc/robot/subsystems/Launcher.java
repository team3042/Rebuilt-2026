package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

@Logged
public class Launcher extends SubsystemBase {
  private final SparkMax flywheelMotor;
  private final SparkMax feederMotor;
  private final SparkMax spindexerMotor;
  private boolean atSP = true;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.KS_VOLTS, LauncherConstants.KV_VOLTS, LauncherConstants.KA_VOLTS);
  private final PIDController m_shooterFeedback = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);

  /** The shooter subsystem for the robot. */
  public Launcher() {

   flywheelMotor = new SparkMax(Constants.MotorIDs.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
   feederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);
   spindexerMotor = new SparkMax(Constants.MotorIDs.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

    m_shooterFeedback.setTolerance(LauncherConstants.KSHOOTER_TOLERANCE_RPS);

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  flywheelMotor.disable();
                  feederMotor.disable();
                  spindexerMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  // public Command shootCommand(double setpointRotationsPerSecond) {
  //   return parallel(
  //           // Run the shooter flywheel at the desired setpoint using feedforward and feedback
  //           run(
  //               () -> {
  //                 flywheelMotor.set(0.4);
  //               }),

  //           // Wait until the shooter has reached the setpoint, and then run the feeder
  //           waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> powerToFeederAndSpindexer()))
  //           .withName("Shoot");
  // }
  public Command shootCommand(double setpointRotationsPerSecond) {

    double power = m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              getFlywheelVelocity(), setpointRotationsPerSecond);

    SmartDashboard.putNumber("Flywheel Motor Power", power);
    SmartDashboard.putNumber("Flywheel Set Point RPS", setpointRotationsPerSecond);
    System.out.println("We are in the wrong launcher command");
    
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  flywheelMotor.set(power);
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitSeconds(1.0).andThen(() -> powerToFeederAndSpindexer())
            .withName("ShootFromLauncher"));
  }

  // // launcherTime is in seconds, runs the launcher for a specified amount of time "launcherTime"
  // public Command shootForTimeCommand(double setpointRotationsPerSecond, double launcherTime) {
  //   return shootCommand(setpointRotationsPerSecond).withTimeout(launcherTime);
  // }

  // launcherTime is in seconds, runs the launcher for the specified amount of time
  public Command shootForTimeCommand2(double setpointRotationsPerSecond, double launcherTime) {

    double power = m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              getFlywheelVelocity(), setpointRotationsPerSecond);

    SmartDashboard.putNumber("Flywheel Motor Power", power);
    SmartDashboard.putNumber("Flywheel Set Point RPS", setpointRotationsPerSecond);
    SmartDashboard.putNumber("launcherTime", launcherTime);
    System.out.println("Timeout");
    System.out.println(launcherTime);
    
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  flywheelMotor.set(power);
                }).withTimeout(launcherTime),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitSeconds(1.0).andThen(() -> powerToFeederAndSpindexer()).withTimeout(launcherTime).andThen(() -> {this.stopLauncherMotors();}))
            .withName("ShootFromLauncher");
  }

  public Command shootCommand2() {
    return parallel(

      run(() -> flywheelMotor.set(0.5)),
      run(() -> powerToFeederAndSpindexer()));
  }

  public void powerToFlywheelMotor(double power) {
    flywheelMotor.set(power);
  }

  public void powerToFeederAndSpindexer() {

      feederMotor.set(-0.4);
      spindexerMotor.set(0.8);
  }

  public void stopLauncherMotors() {
    feederMotor.set(0);
    spindexerMotor.set(0);
    flywheelMotor.set(0);
  }

  public double getFlywheelVelocity() {
    return flywheelMotor.getEncoder().getVelocity()/60d;
  }

  public double getTrueFlywheelVelocity() {
    return flywheelMotor.getEncoder().getVelocity();
  }
}