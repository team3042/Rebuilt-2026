package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

@Logged
public class Launcher extends SubsystemBase {
  private final SparkMax flywheelMotor;
  private final SparkMax feederMotor;
  private final SparkMax spindexerMotor;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.KS_VOLTS, LauncherConstants.KV_VOLTS_SECONDS_PER_ROTATION, LauncherConstants.KA_VOLTS);
  private final PIDController m_shooterFeedback = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);

  /** The shooter subsystem for the robot. */
  public Launcher() {

   flywheelMotor = new SparkMax(Constants.MotorIDs.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
   feederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);
   spindexerMotor = new SparkMax(Constants.MotorIDs.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

    m_shooterFeedback.setTolerance(LauncherConstants.KSHOOTER_TOLERALCE_RPS);

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
  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  flywheelMotor.set(
                      m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                          + m_shooterFeedback.calculate(
                              flywheelMotor.getEncoder().getVelocity(), setpointRotationsPerSecond));
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> powerToFeederAndSpindexer()))
            .finallyDo(interrupted -> {
               // runs when command ends OR is interrupted
          stopLauncherMotors();
      })
      .withName("Shoot");
  }

  private void powerToFeederAndSpindexer() {

      feederMotor.set(1);
      spindexerMotor.set(1);
  }

  public void stopLauncherMotors() {
    feederMotor.set(0);
    spindexerMotor.set(0);
    flywheelMotor.set(0);
  }


}