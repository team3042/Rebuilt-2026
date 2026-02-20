 package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

 public class Flywheel extends SubsystemBase {

   private static SparkMax flywheelMotor;
   private static RelativeEncoder flywheelEncoder;

   public Flywheel() {
      flywheelMotor = new SparkMax(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
      flywheelEncoder = flywheelMotor.getEncoder();
   }
      public void powerToFlywheel(double power){
         flywheelMotor.set(power);
      }

      public void stopFlywheel(){
         flywheelMotor.set(0);
      } 
      public double getVeloity(){
         return flywheelEncoder.getVelocity(); 
      }


   }
