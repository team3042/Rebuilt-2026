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

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 2;
        public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 3;

        public static final int TURRET_START_LIMIT_SWITCH_ID = 1;
        public static final int TURRET_END_LIMIT_SWITCH_ID = 0;

        public static final int CLIMBER_LIMIT_SWITCH_ID = 4;
        
    }
    
    public static class PowerConstants {
        
        public static final double INTAKE_RUN_POWER = -0.8;
        public static final double INTAKE_REVERSE_RUN_POWER = 0.5;
        public static final double INTAKE_POSITION_IN_POWER = 0.15;
        public static final double INTAKE_POSITION_OUT_POWER = -0.15;

        public static final double TURRET_MOTOR_POWER_LEFT = -0.5;
        public static final double TURRET_MOTOR_POWER_RIGHT = 0.5;

        public static final double FLYWHEEL_POWER = 0.5;

        public static final double FEEDER_RUN_TO_LAUNCHER_POWER = 0.5;
        public static final double FEEDER_RUN_AWAY_FROM_LAUNCHER_POWER = -0.5;
    }

    public static class LauncherConstants {

        //TODO: find kSVolts
        public static double KS_VOLTS = 0.1;
        public static double KV_VOLTS_SECONDS_PER_ROTATION = 0.38;
        public static double KA_VOLTS = 0.09;
        // PID gains
        public static double kP = 0.002;
        public static double KSHOOTER_TOLERANCE_RPS = 5;
        public static double kI = 0;
        public static double kD = 0;
        public static double DESIRED_RPS = 30;
    }
}
