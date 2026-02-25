package frc.robot;

import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

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

    public static class VisionConstants {

        public static final String CAMERA_NAME = "photonvision";
        public static final PoseStrategy FALLBACK_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
        public static final AprilTagFieldLayout PRIMARY_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR.;
        public static final List<VisionTargetSim> SIM_TARGETS = FIELD_CONSTANTS.APRIL_TAG_LAYOUT.getTags().stream().map((AprilTag tag) -> {
      return new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.ID);
    }).toList();

    }

    public static class FIELD_CONSTANTS {

    public static final AprilTagFields APRIL_TAG_FIELD =
      AprilTagFields.k2025ReefscapeAndyMark;

    public static final AprilTagFieldLayout HOME_FIELD_LAYOUT =
      Utility.makeHomeField();

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
      HOME_FIELD_LAYOUT;

    public static final Distance FIELD_LENGTH = Units.Meters.of(APRIL_TAG_LAYOUT.getFieldLength());
    public static final Distance FIELD_WIDTH = Units.Meters.of(
      APRIL_TAG_LAYOUT.getFieldWidth()
    );

    // how close the estimated pose can get to the field border before we invalidate it
    public static final Distance FIELD_BORDER_MARGIN = Units.Inches.of(0.1);

    // how far off on the z axis the estimated pose can be before we invalidate it
    public static final Distance Z_MARGIN = Units.Feet.of(0.5);
  }
}
