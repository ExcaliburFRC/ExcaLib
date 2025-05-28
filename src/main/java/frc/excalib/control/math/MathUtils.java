package frc.excalib.control.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

/**
 * Utility class for mathematical operations and geometry-related calculations.
 */
public class MathUtils {

    /**
     * Ensures that the absolute value of the given number does not exceed the specified limit.
     * @param value     The value to be limited.
     * @param sizeLimit The maximum allowable absolute value.
     * @return The value limited to the specified maximum absolute value, preserving its sign.
     */
    public static double minSize(double value, double sizeLimit) {
        return Math.min(sizeLimit, Math.abs(value)) * Math.signum(value);
    }

    /**
     * Limits the given value to the specified limit. If the value exceeds the limit, it is clamped to the limit.
     * @param value The value to be limited.
     * @param limit The maximum allowable value (positive or negative).
     * @return The value clamped to the specified limit.
     */
    public static double limitTo(double value, double limit) {
        if ((limit > 0 && limit < value) || (limit < 0 && limit > value)) {
            return limit;
        }
        return value;
    }

    /**
     * Limits a value in continuous system
     * @param value the value to limit
     * @param referencePoint the point around which to constrain the value within ±π (180 degrees)
     * @return if the value and the current position are equals - empty optional,
     * else the limited values in range of maximum 180 degrees (in every direction) from the current position
     */
    public static Optional<Pair<Double, Double>> continuousLimit(double value, double referencePoint) {
        double upperLimitedValue, lowerLimitedValue;
        if (value == referencePoint) { // if the value equals to the current position, return an empty optional
            return Optional.empty();
        }
        if (value > referencePoint) {
            // if the value is greater than the current position, subtract from it a full rotation each time until it's less than a rotation from the current position
            upperLimitedValue = value;
            while ((upperLimitedValue - 2 * Math.PI) > referencePoint) {
                upperLimitedValue -= 2 * Math.PI;
            }
            lowerLimitedValue = upperLimitedValue - 2 * Math.PI;
            return Optional.of(Pair.of(upperLimitedValue, lowerLimitedValue));
        }
        // if the value is smaller than the current position, add to it a full rotation each time until it's less than a rotation from the current position
        lowerLimitedValue = value;
        while ((lowerLimitedValue + 2 * Math.PI) < referencePoint) {
            lowerLimitedValue += 2 * Math.PI;
        }
        upperLimitedValue = lowerLimitedValue + 2 * Math.PI;
        return Optional.of(Pair.of(upperLimitedValue, lowerLimitedValue));
    }

    /**
     * Calculates the optimal target position for a robot to reach a target while avoiding a circular obstacle.
     * @param robot          The current position of the robot as a Translation2d.
     * @param target         The target position as a Translation2d.
     * @param obstacleCenter The center of the obstacle as a Translation2d.
     * @return The optimal target position as a Translation2d.
     */
    public static Translation2d getTargetPose(Translation2d robot, Translation2d target, Translation2d obstacleCenter) {
        double radius = obstacleCenter.getDistance(target);
        Circle c = new Circle(obstacleCenter.getX(), obstacleCenter.getY(), radius);
        Line[] tangents = c.getTangents(robot);
        Rotation2d alpha = target.minus(obstacleCenter).getAngle();
        Rotation2d theta = robot.minus(obstacleCenter).getAngle();

        // If the angle between the target and robot is less than 60 degrees, return the target directly.
        if (Math.abs(alpha.minus(theta).getRadians()) < Math.PI / 3) {
            return target;
        }

        // If no tangents exist, calculate a point on the obstacle perimeter and find the intersection.
        if (tangents.length == 0) {
            Translation2d onPerimeter = obstacleCenter.plus(new Translation2d(radius, robot.minus(obstacleCenter).getAngle()));
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