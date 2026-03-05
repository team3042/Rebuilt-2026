package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
        
    }
    
    public static class PowerConstants {
        
        public static final double INTAKE_RUN_POWER = -0.8;
        public static final double INTAKE_REVERSE_RUN_POWER = 0.5;
        public static final double INTAKE_POSITION_POWER = 0.15;

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
        // public static double kP = 1.12; // this might be too high
        public static double kP = 0.012;
        public static double KSHOOTER_TOLERANCE_RPS = 0.05;
        public static double KA_VOLTS = 0.09;
        public static double kI = 0;
        public static double kD = 0;
        public static double DESIRED_RPS = 10;

    }
    public static class FieldConstants {

        public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final double FIELD_LENGTH = FIELD_LAYOUT.getFieldLength();
        public static final double FIELD_WIDTH = FIELD_LAYOUT.getFieldWidth();

        public static class Hub {

            
            // Dimensions
            public static final double WIDTH =
                edu.wpi.first.math.util.Units.inchesToMeters(47.0);
            public static final double HEIGHT =
                edu.wpi.first.math.util.Units.inchesToMeters(72.0); // includes the catcher at the top
            public static final double INNER_WIDTH =
                edu.wpi.first.math.util.Units.inchesToMeters(41.7);
            public static final double INNER_HEIGHT =
                edu.wpi.first.math.util.Units.inchesToMeters(56.5);

            // Relevant reference points on alliance side
            public static final Translation3d blueTopCenterPoint = new Translation3d(
                FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0,
                HEIGHT
            );
            public static final Translation3d blueInnerCenterPoint = new Translation3d(
                FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0,
                INNER_HEIGHT
            );

            public static final Translation2d nearLeftCorner = new Translation2d(
                blueTopCenterPoint.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0
            );
            public static final Translation2d nearRightCorner = new Translation2d(
                blueTopCenterPoint.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0
            );
            public static final Translation2d farLeftCorner = new Translation2d(
                blueTopCenterPoint.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0
            );
            public static final Translation2d farRightCorner = new Translation2d(
                blueTopCenterPoint.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0
            );

            // Relevant reference points on the opposite side
            public static final Translation3d redTopCenterPoint = new Translation3d(
                FIELD_LAYOUT.getTagPose(4).get().getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0,
                HEIGHT
            );
            public static final Translation2d redNearLeftCorner = new Translation2d(
                redTopCenterPoint.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0
            );
            public static final Translation2d redNearRightCorner = new Translation2d(
                redTopCenterPoint.getX() - WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0
            );
            public static final Translation2d redFarLeftCorner = new Translation2d(
                redTopCenterPoint.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 + WIDTH / 2.0
            );
            public static final Translation2d redFarRightCorner = new Translation2d(
                redTopCenterPoint.getX() + WIDTH / 2.0,
                FIELD_WIDTH / 2.0 - WIDTH / 2.0
            );

            // Hub faces
            public static final Pose2d nearFace = FIELD_LAYOUT.getTagPose(26)
                .get()
                .toPose2d();
            public static final Pose2d farFace = FIELD_LAYOUT.getTagPose(20)
                .get()
                .toPose2d();
            public static final Pose2d rightFace = FIELD_LAYOUT.getTagPose(18)
                .get()
                .toPose2d();
            public static final Pose2d leftFace = FIELD_LAYOUT.getTagPose(21)
                .get()
                .toPose2d();
        }
    }
}
