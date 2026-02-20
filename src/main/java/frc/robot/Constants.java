package frc.robot;

public class Constants {

    public static final double flyWheelVelocityThreshold = 10;
    
    // put all motor IDs in this class
    public static class MotorIDs {

        public static final int SPIDEXER_MOTOR_ID = 101;
        public static final int FEEDER_MOTOR_ID = 100;
        public static final int INTAKE_POSITION_MOTOR_ID = 0;
        public static final int FLYWHEEL_MOTOR_ID = 10;

    }

    public static class DigitalIO {

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 0;
        public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 1;

    }

    public static class PowerConstants {

        public static final double FEEDER_RUN_TO_LAUNCHER_POWER = 0.5;
        public static final double FEEDER_RUN_AWAY_FROM_LAUNCHER_POWER = -0.5;

    }
}
