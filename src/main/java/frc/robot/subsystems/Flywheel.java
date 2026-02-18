 package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

 public class Flywheel {

   private static SparkMax flywheelMotor;

   public Flywheel() {
      flywheelMotor = new SparkMax(18, MotorType.kBrushless);
   }
      public static void shoot(double power){
         flywheelMotor.set(power);
      }

      public void stopShoot(){
         flywheelMotor.set(0);
      } 


   }












    
 
