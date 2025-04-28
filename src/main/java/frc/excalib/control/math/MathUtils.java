package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility class for mathematical operations and geometry-related calculations.
 */
public class MathUtils {

    /**
     * Ensures that the absolute value of the given number does not exceed the specified limit.
     *
     * @param val       The value to be limited.
     * @param sizeLimit The maximum allowable absolute value.
     * @return The value limited to the specified maximum absolute value, preserving its sign.
     */
    public static double minSize(double val, double sizeLimit) {
        return Math.min(sizeLimit, Math.abs(val)) * Math.signum(val);
    }

    /**
     * Limits the given value to the specified limit. If the value exceeds the limit, it is clamped to the limit.
     *
     * @param limit The maximum allowable value (positive or negative).
     * @param value The value to be limited.
     * @return The value clamped to the specified limit.
     */
    public static double limitTo(double limit, double value) {
        if ((limit > 0 && limit < value) || (limit < 0 && limit > value)) {
            return limit;
        }
        return value;
    }

    /**
     * Calculates the optimal target position for a robot to reach a target while avoiding a reef.
     *
     * @param robot      The current position of the robot as a Translation2d.
     * @param target     The target position as a Translation2d.
     * @param reefCenter The center of the reef as a Translation2d.
     * @return The optimal target position as a Translation2d.
     */
    public static Translation2d getTargetPose(Translation2d robot, Translation2d target, Translation2d reefCenter) {
        double radius = reefCenter.getDistance(target);
        Circle c = new Circle(reefCenter.getX(), reefCenter.getY(), radius);
        Line[] tangents = c.getTangents(robot);
        Rotation2d alpha = target.minus(reefCenter).getAngle();
        Rotation2d theta = robot.minus(reefCenter).getAngle();

        // If the angle between the target and robot is less than 60 degrees, return the target directly.
        if (Math.abs(alpha.minus(theta).getRadians()) < Math.PI / 3) {
            return target;
        }

        // If no tangents exist, calculate a point on the reef perimeter and find the intersection.
        if (tangents.length == 0) {
            Translation2d onPerimeter = reefCenter.plus(new Translation2d(radius, robot.minus(reefCenter).getAngle()));
            Line tangent = c.getTangent(onPerimeter);
            return tangent.findIntersection(c.getTangent(target));
        }

        // If tangents exist, find the intersection of the target tangent with the robot tangents.
        Line targetTangent = c.getTangent(target);
        if (tangents.length == 1) {
            return targetTangent.findIntersection(tangents[0]);
        }

        // Calculate the two possible intersection points and return the closer one to the target.
        Translation2d translation1 = targetTangent.findIntersection(tangents[0]);
        Translation2d translation2 = targetTangent.findIntersection(tangents[1]);

        if (target.getDistance(translation1) < target.getDistance(translation2)) {
            return translation1;
        }
        return translation2;
    }
}