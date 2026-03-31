package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

    // put all motor IDs in this class
    public static class MotorIDs {

        public static final int INTAKE_RUN_MOTOR_ID = 21;
        public static final int INTAKE_POSITION_MOTOR_ID = 26;
        public static final int SPINDEXER_MOTOR_ID = 23;
        public static final int FEEDER_MOTOR_ID = 24;
        public static final int FLYWHEEL_MOTOR_ID = 22;

    }

    public static class DigitalIO {

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 3;
        //public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 4;
        
    }
    
    public static class PowerConstants {
        
        public static final double INTAKE_RUN_POWER = -0.8;
        public static final double INTAKE_REVERSE_RUN_POWER = 0.5;
        public static final double INTAKE_POSITION_IN_POWER = 0.15;
        public static final double INTAKE_POSITION_OUT_POWER = -0.15;

        public static final double FEEDER_RUN_TO_LAUNCHER_POWER = 0.5;
        public static final double FEEDER_RUN_AWAY_FROM_LAUNCHER_POWER = -0.5;
    }

    public static class LauncherConstants {

        //TODO: find kSVolts
        public static double KS_VOLTS = 0;
        public static double KV_VOLTS = 0.0097;
        public static double KA_VOLTS = 0.01;
        // PID gains
        public static double kP = 0.001;
        public static double KSHOOTER_TOLERANCE_RPS = 54;
        public static double kI = 0;
        public static double kD = 0;
        public static double DESIRED_RPS = 54;
    }

    public static class Vision {
        // TO-DO: Set the correct height ("z") for both cameras in the Translation3d on lines 21 and 26 
        public static final String CAMERA_NAME_1 = "Back";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam_1 =  // x=10.13", y=11.5", guessing z at 18
                new Transform3d(new Translation3d(0.257, 0.292, 0.457), new Rotation3d(0, 0, Units.degreesToRadians(180)));

        public static final String CAMERA_NAME_2 = "Side";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam_2 = // x=11.5", y=10.13", guessing z at 16.5
                new Transform3d(new Translation3d(0.292, 0.257, 0.419), new Rotation3d(0, 0, Units.degreesToRadians(90)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

}
