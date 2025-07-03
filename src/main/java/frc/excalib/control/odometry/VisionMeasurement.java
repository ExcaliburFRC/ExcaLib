package frc.excalib.control.odometry;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionMeasurement(Pose2d estimatedRobotPose, double timestampSeconds) {}
