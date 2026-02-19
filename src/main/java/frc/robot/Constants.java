package frc.robot;

public class Constants {

    // put all motor IDs in this class
    public static class MotorIDs {

        public static final int INTAKE_POSITION_MOTOR_ID = 0;
        public static final int SPIDEXER_MOTOR_ID = 1;
        public static final int FEEDER_MOTOR_ID = 2;
        public static final int TURRET_MOTOR_ID = 3;
    }

    public static class DigitalIO {

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 0;
        public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 1;
        public static final int TURRET_ZERO_LIMIT_SWITCH_ID = 2;
        public static final int TURRET_ONE_EIGHTY_LIMIT_SWITCH_ID = 3;

    }

    public static class PowerConstants {

        public static final double TURRET_MOTOR_POWER_LEFT = -0.5;
        public static final double TURRET_MOTOR_POWER_RIGHT = 0.5;
    }
}
