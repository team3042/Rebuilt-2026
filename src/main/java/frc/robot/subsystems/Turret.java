package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final SparkMax TurretMotor;
  public final DigitalInput ZeroLimitSwitch;
  public final DigitalInput OneEightyLimitSwitch;

  public Turret() {

    TurretMotor = new SparkMax(Constants.MotorIDs.TURRET_MOTOR_ID, MotorType.kBrushless);
    ZeroLimitSwitch = new DigitalInput(Constants.DigitalIO.TURRET_ZERO_LIMIT_SWITCH_ID);
    OneEightyLimitSwitch = new DigitalInput(Constants.DigitalIO.TURRET_ONE_EIGHTY_LIMIT_SWITCH_ID);
  }

  public void PowerToTurret(double percentPower){
    //left is negative, right is positive
    if ((ZeroLimitSwitch.get() || percentPower > 0) && (OneEightyLimitSwitch.get() || percentPower < 0)) {
      TurretMotor.set(percentPower);
    } else {
      TurretMotor.set(0);
    }
  }

  public void stopTurret(){
    TurretMotor.set(0);
  }

  public double getEncoderCounts(){
    return TurretMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
