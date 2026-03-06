package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final SparkMax TurretMotor;
  public final DigitalInput startLimitSwitch;
  public final DigitalInput endLimitSwitch;

  public Turret() {

    TurretMotor = new SparkMax(Constants.MotorIDs.TURRET_MOTOR_ID, MotorType.kBrushless);
    startLimitSwitch = new DigitalInput(Constants.DigitalIO.TURRET_START_LIMIT_SWITCH_ID);
    endLimitSwitch = new DigitalInput(Constants.DigitalIO.TURRET_END_LIMIT_SWITCH_ID);
  }

  public void powerToTurret(double percentPower) {
    // left is negative, right is positive
    if ((startLimitSwitch.get() || percentPower > 0) && (endLimitSwitch.get() || percentPower < 0)) {
      TurretMotor.set(percentPower);
    } else {
      TurretMotor.set(0);
    }
  }

  public void stopTurret() {
    TurretMotor.set(0);
  }

  public double getEncoderCounts() {
    return TurretMotor.getEncoder().getPosition();
  }

  public double getAngle() {
    double encoderCounts = TurretMotor.getEncoder().getPosition();
    return encoderCounts / 7.6667;
  }

  public static double angleToEncoderCount(double angle) {
    return angle * 7.6667;
  }

  public static double encoderCountToAngle(double angle) {
    return angle / 7.6667;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
