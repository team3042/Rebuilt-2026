 package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants; 
 public class Flywheel {

   private static SparkMax flywheelMotor;
   private static RelativeEncoder flywheelEncoder;

   public Flywheel() {
      flywheelMotor = new SparkMax(Constants.MotorIDs.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
      flywheelEncoder = flywheelMotor.getEncoder();
   }
      public static void shoot(double power){
         flywheelMotor.set(power);
      }

      public void stopShoot(){
         flywheelMotor.set(0);
      } 
      public double getSpeed(){
         return flywheelEncoder.getVelocity();
      }


   }
