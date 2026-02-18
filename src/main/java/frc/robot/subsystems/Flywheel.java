 package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

 public class Flywheel {

   private static SparkMax flywheelMotor;
   private static RelativeEncoder flywheelEncoder;

   public Flywheel() {
      flywheelMotor = new SparkMax(18, MotorType.kBrushless);
      flywheelEncoder = flywheelMotor.getEncoder();
   }
      public void shoot(double power){
         flywheelMotor.set(power);
      }

      public void stopShoot(){
         flywheelMotor.set(0);
      } 
      public double getSpeed(){
         return flywheelEncoder.getVelocity(); 
      }


   }
