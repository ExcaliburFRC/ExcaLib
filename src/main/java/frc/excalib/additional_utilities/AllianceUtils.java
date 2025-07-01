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
 * the purpose of this class is to supply different utility functions
 * for functionality that depends on the robot alliance.
 */
public class AllianceUtils {
    public static Field field = Field.REEFSCAPE_WELDED;

    public static void setField(Field newField) {
        field = newField;
    }

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
     * Converts a pose to a new pose relative to the current driver station alliance by rotating it if needed.
     *
     * @param pose the current blue alliance pose
     * @return the converted Pose2d pose
     */
    public static Pose2d rotateToAlliancePose(Pose2d pose) {
        return isBlueAlliance() ?
                pose :
                new Pose2d(
                        field.m_lengthMeters - pose.getX(),
                        field.m_widthMeters - pose.getY(),
                        pose.getRotation().minus(Rotation2d.kPi)
                );
    }

    /**
     * Converts a pose to a new pose relative to the current driver station alliance by mirroring it if needed.
     *
     * @param pose the current blue alliance pose
     * @return the converted Pose2d pose
     */
    public static Pose2d mirrorToAlliancePose(Pose2d pose) {
        return isBlueAlliance() ?
                pose :
                new Pose2d(
                        field.m_lengthMeters - pose.getX(),
                        pose.getY(),
                        Rotation2d.kPi.minus(pose.getRotation())
                );
    }

    /**
     * A class represents a pose in the current alliance.
     */
    public static class AlliancePose {
        private final Pose2d m_pose;

        /**
         * @param translation the translation component
         * @param rotation    the rotation component
         */
        public AlliancePose(Translation2d translation, Rotation2d rotation) {
            this.m_pose = new Pose2d(translation, rotation);
        }

        /**
         * @param x               the x component
         * @param y               the y component
         * @param rotationRadians the rotation component in radians
         */
        public AlliancePose(double x, double y, double rotationRadians) {
            this(new Translation2d(x, y), new Rotation2d(rotationRadians));
        }

        /**
         * @param pose the pose
         */
        public AlliancePose(Pose2d pose) {
            this(pose.getTranslation(), pose.getRotation());
        }

        /**
         * A constructor that creates a pose with zero x and y components
         *
         * @param rotationRadians the rotation component in radians
         */
        public AlliancePose(double rotationRadians) {
            this(new Translation2d(0, 0), new Rotation2d(rotationRadians));
        }

        /**
         * An empty constructor that creates a pose with 0 values.
         */
        public AlliancePose() {
            this(0, 0, 0);
        }

        /**
         * @return the original pose if the robot is blue, the rotated pose if the robot is red
         */
        public Pose2d getRotated() {
            return rotateToAlliancePose(m_pose);
        }

        /**
         * @return the original pose if the robot is blue, the mirrored pose if the robot is red
         */
        public Pose2d getMirrored() {
            return mirrorToAlliancePose(m_pose);
        }
    }

    public enum Field {
        REEFSCAPE_WELDED(17.548, 8.052),
        REEFSCAPE_ANDYMARK(17.548, 8.042);

        public final double m_lengthMeters, m_widthMeters;

        Field(double lengthMeters, double widthMeters) {
            this.m_lengthMeters = lengthMeters;
            this.m_widthMeters = widthMeters;
        }
    }
}
