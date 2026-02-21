package frc.robot;

public class Constants {

   // put all motor IDs in this class
    public static class MotorIDs {

        public static final int INTAKE_RUN_MOTOR_ID = 21;
        public static final int INTAKE_POSITION_MOTOR_ID = 22;
        public static final int SPINDEXER_MOTOR_ID = 23;
        public static final int FEEDER_MOTOR_ID = 24;
        public static final int FLYWHEEL_MOTOR_ID = 25;
        public static final int TURRET_MOTOR_ID = 26;
        public static final int HOOD_MOTOR_ID = 27;
        public static final int CLIMBER_MOTOR_ID = 28;

    }

    public static class DigitalIO {

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 0;
        public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 1;

        public static final int TURRET_START_LIMIT_SWITCH_ID = 2;
        public static final int TURRET_END_LIMIT_SWITCH_ID = 3;
        
    }
    
    public static class PowerConstants {
        
        public static final double INTAKE_RUN_POWER = 0.5;
        public static final double INTAKE_REVERSE_RUN_POWER = -0.5;
        public static final double INTAKE_POSITION_POWER = 0.5;

        public static final double TURRET_MOTOR_POWER_LEFT = -0.5;
        public static final double TURRET_MOTOR_POWER_RIGHT = 0.5;

        public static final double FLYWHEEL_POWER = 0.5;

        public static final double FEEDER_RUN_TO_LAUNCHER_POWER = 0.5;
        public static final double FEEDER_RUN_AWAY_FROM_LAUNCHER_POWER = -0.5;
    }

    public static class LauncherConstants {

        //TODO: find kSVolts
        public static double kSVolts = 0;
        public static double kVVoltSecondsPerRotation = 0.51;
        public static double kP = 1.12;
        public static double kShooterToleranceRPS = 0.05;
        public static double kAVolts = 0.07;
        public static double kI = 0;
        public static double kD = 0;


    }
}
