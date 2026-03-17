package frc.excalib.additional_utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

/**
 * the purpose of this class is to supply different utility functions for functionality
 * that depends on the robot alliance.
 *
 * @author Shai Grossman
 */
public class AllianceUtils {
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.07;

    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DS alliance empty!", false);
        return true;
    }

    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    public static int getdirection() {
        if (isBlueAlliance())
            return 1;
        return -1;
    }

    public static Translation2d toAlliancePose(Translation2d bluePose) {
        if (isBlueAlliance()) return bluePose;
        return switchAlliance(new Pose2d(bluePose, new Rotation2d())).getTranslation();
    }


    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param bluePose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d bluePose) {
        if (isBlueAlliance()) return bluePose;
        return switchAlliance(bluePose);
    }

    public static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(), FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
        );
    }

    public static Pose2d mirrorAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    public static class AlliancePose {
        private Pose2d pose;

        public AlliancePose(double x, double y, double degrees) {
            this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        }

        public AlliancePose() {
            this.pose = new Pose2d(new Translation2d(), new Rotation2d());
        }

        public AlliancePose(Translation2d translation, Rotation2d rotation) {
            this.pose = new Pose2d(translation, rotation);
        }

        public AlliancePose(double degrees) {
            this.pose = new Pose2d(0, 0, Rotation2d.fromDegrees(degrees));
        }

        public Pose2d get() {
            return toAlliancePose(pose);
        }
    }}
