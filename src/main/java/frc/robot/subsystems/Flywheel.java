package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

@Logged
public class Flywheel extends SubsystemBase {
  private final SparkMax flywheelMotor;
  private final SparkMax feederMotor;
  private final SparkMax spindexerMotor;

//   private final Encoder m_shooterEncoder =
//       new Encoder(
//           LauncherConstants.kEncoderPorts[0],
//           LauncherConstants.kEncoderPorts[1],
//           LauncherConstants.kEncoderReversed);
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.kSVolts, LauncherConstants.kVVoltSecondsPerRotation);
  private final PIDController m_shooterFeedback = new PIDController(LauncherConstants.kP, 0.0, 0.0);

  /** The shooter subsystem for the robot. */
  public Flywheel() {

   flywheelMotor = new SparkMax(Constants.MotorIDs.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
   feederMotor = new SparkMax(Constants.MotorIDs.FEEDER_MOTOR_ID, MotorType.kBrushless);
   spindexerMotor = new SparkMax(Constants.MotorIDs.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

    m_shooterFeedback.setTolerance(LauncherConstants.kShooterToleranceRPS);
   //  m_shooterEncoder.setDistancePerPulse(LauncherConstants.kEncoderDistancePerPulse);

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
        .withName("Shoot");
  }

  private void powerToFeederAndSpindexer() {

      feederMotor.set(1);
      spindexerMotor.set(1);
  }


}