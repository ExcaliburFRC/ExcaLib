/**
 * © 2025 Excalibur FRC. All rights reserved.
 * This file is part of ExcaLIb and may not be copied, modified,
 * or distributed without permission, except as permitted by license.
 * learn more at - https://github.com/ExaliburFRC/ExcaLIb
 */

package frc.excalib.additional_utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

/**
 * the purpose of this class is too supply different utility functions for functionality
 * that depends on the robot alliance.
 * @author Shai Grossman
 */
public class AllianceUtils {
    public static final double FIELD_LENGTH_METERS = 17.548;
    public static final double FIELD_WIDTH_METERS = 8.052;

    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DriverStation Alliance is Empty!", false);
        return true;
    }

    /**
     * @return whether the robot is on the red alliance
     */
    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted Pose2d pose
     */
    public static Pose2d rotateToAlliancePose(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(), FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
        );
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted Pose2d pose
     */
    public static Pose2d mirrorToAlliancePose(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    public static class AlliancePose {
        private final Pose2d m_pose;

        public AlliancePose(double x, double y, double rotationRadians) {
            this.m_pose = new Pose2d(x, y, new Rotation2d(rotationRadians));
        }

        public AlliancePose(Translation2d translation, Rotation2d rotation) {
            this.m_pose = new Pose2d(translation, rotation);
        }

        public AlliancePose(Pose2d pose) {
            this.m_pose = new Pose2d(pose.getTranslation(), pose.getRotation());
        }

        public AlliancePose(double rotationRadians) {
            this.m_pose = new Pose2d(0, 0, new Rotation2d(rotationRadians));
        }

        public Pose2d getRotated() {
            return rotateToAlliancePose(m_pose);
        }

        public Pose2d getMirrored() {
            return mirrorToAlliancePose(m_pose);
        }
    }
}
