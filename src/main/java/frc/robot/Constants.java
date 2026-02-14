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

        public static final int INTAKE_POSITION_MOTOR_ID = 0;
        public static final int SPIDEXER_MOTOR_ID = 1;
        public static final int FEEDER_MOTOR_ID = 2;

    }

    public static class DigitalIO {

        public static final int INTAKE_INSIDE_LIMIT_SWITCH_ID = 0;
        public static final int INTAKE_OUTSIDE_LIMIT_SWITCH_ID = 1;

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
    );
    public static final Distance FIELD_WIDTH = Units.Meters.of(
      APRIL_TAG_LAYOUT.getFieldWidth()
    );

    // how close the estimated pose can get to the field border before we invalidate it
    public static final Distance FIELD_BORDER_MARGIN = Units.Inches.of(0.1);

    // how far off on the z axis the estimated pose can be before we invalidate it
    public static final Distance Z_MARGIN = Units.Feet.of(0.5);
  }
}
