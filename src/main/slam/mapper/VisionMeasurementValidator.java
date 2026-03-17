// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.slam.mapper;

import edu.wpi.first.math.geometry.Pose2d;
import frc.excalib.additional_utilities.AllianceUtils;

/**
 * Utility class for validating vision measurements before using them in odometry updates.
 * Helps prevent corrupted pose data from being added to the odometry system.
 */
public class VisionMeasurementValidator {
    // Thresholds for rejecting unreliable vision measurements
    private static final double TRANSLATION_THRESHOLD = 0.01; // meters
    private static final double ROTATION_THRESHOLD = 0.01; // radians (~0.57 degrees)
    private static final double MAX_TRANSLATION_PER_FRAME = 5.0; // meters (max reasonable movement per frame)
    private static final double MAX_ROTATION_PER_FRAME = Math.PI; // radians

    /**
     * Validates whether a vision measurement is reliable and should be used for odometry updates.
     *
     * @param visionPose The pose from the vision system
     * @param lastValidPose The last known valid pose (for continuity checking)
     * @return true if the pose is valid and should be used, false otherwise
     */
    public static boolean isValidVisionMeasurement(Pose2d visionPose, Pose2d lastValidPose) {
        // Check for null pose
        if (visionPose == null) {
            return false;
        }

        // Check for zero/default pose (common vision failure mode)
        if (isDefaultPose(visionPose)) {
            return false;
        }

        // If we have a previous pose, check continuity
        if (lastValidPose != null) {
            return checkPoseContinuity(visionPose, lastValidPose);
        }

        return true;
    }

    /**
     * Checks if a pose is the default/uninitialized pose (0, 0, 0)
     */
    private static boolean isDefaultPose(Pose2d pose) {
        return Math.abs(pose.getX()) < TRANSLATION_THRESHOLD &&
               Math.abs(pose.getY()) < TRANSLATION_THRESHOLD &&
               Math.abs(pose.getRotation().getRadians()) < ROTATION_THRESHOLD;
    }

    /**
     * Checks if movement from lastPose to newPose is physically reasonable
     */
    private static boolean checkPoseContinuity(Pose2d newPose, Pose2d lastPose) {
        // Check translation continuity
        double translationDelta = lastPose.getTranslation().getDistance(newPose.getTranslation());
        if (translationDelta > MAX_TRANSLATION_PER_FRAME) {
            return false;
        }

        // Check rotation continuity
        double rotationDelta = Math.abs(newPose.getRotation().minus(lastPose.getRotation()).getRadians());
        // Handle wrap-around
        rotationDelta = Math.min(rotationDelta, 2 * Math.PI - rotationDelta);
        if (rotationDelta > MAX_ROTATION_PER_FRAME) {
            return false;
        }

        return true;
    }

    /**
     * Checks if vision pose is within the field boundaries
     */
    public static boolean isWithinFieldBounds(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -0.5 && x <= AllianceUtils.FIELD_LENGTH_METERS + 0.5 &&
               y >= -0.5 && y <= AllianceUtils.FIELD_WIDTH_METERS + 0.5;
    }
}

