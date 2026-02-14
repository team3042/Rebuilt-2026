 package frc.robot.subsystems;

<<<<<<< HEAD
import com.revrobotics.RelativeEncoder;
=======
>>>>>>> 1817397e9fe6bd97545b0c9d01e100525badebe2
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

 public class Flywheel {

   private static SparkMax flywheelMotor;
<<<<<<< HEAD
   private static RelativeEncoder flywheelEncoder;

   public Flywheel() {
      flywheelMotor = new SparkMax(18, MotorType.kBrushless);
      flywheelEncoder = flywheelMotor.getEncoder();
=======

   public Flywheel() {
      flywheelMotor = new SparkMax(18, MotorType.kBrushless);
>>>>>>> 1817397e9fe6bd97545b0c9d01e100525badebe2
   }
      public static void shoot(double power){
         flywheelMotor.set(power);
      }

      public void stopShoot(){
         flywheelMotor.set(0);
      } 
<<<<<<< HEAD
      public double getSpeed(){
         return flywheelEncoder.getVelocity();
      }


   }
=======


   }












    
 
>>>>>>> 1817397e9fe6bd97545b0c9d01e100525badebe2
