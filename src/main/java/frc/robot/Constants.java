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

    public static class TurretConstants {

        // X is left and right, Y is front to back (in meters)
        // TODO: measure these values more precisely;
        public static final double TURRET_X_OFFSET = 0.1778;
        public static final double TURRET_Y_OFFSET = 0.1397;
        public static final double ROBOT_TO_TURRET_RADIUS = Math.sqrt(TURRET_X_OFFSET*TURRET_X_OFFSET + TURRET_Y_OFFSET*TURRET_Y_OFFSET);

    }

    public static class IntakeConstants {

        public static final double INTAKE_OUT_POSITION = -30; //TODO: find the actual position for the intake to be fully extended

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
        public static double DESIRED_RPS = 54;
    }
}
