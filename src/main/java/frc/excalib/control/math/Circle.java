package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.*;

/**
 * Represents a circle in a 2D plane, defined by its center and radius.
 */
public class Circle {
    // The radius of the circle
    public final double r;
    // The center of the circle as a Translation2d object
    public final Translation2d center;

    /**
     * Constructs a Circle object with the specified center coordinates and radius.
     *
     * @param a The x-coordinate of the circle's center.
     * @param b The y-coordinate of the circle's center.
     * @param r The radius of the circle.
     */
    public Circle(double a, double b, double r) {
        this.center = new Translation2d(a, b);
        this.r = r;
    }

    /**
     * Constructs a Circle object with a Point and radius.
     *
     *  @param center the coordinates of the circle's center
     * @param r The radius of the circle.
     */
    public Circle(Point center, double r){
        this.center = new Translation2d(center.x, center.y);
        this.r = r;
    }

    /**
     * Calculates the tangent line to the circle at a given point on its circumference.
     *
     * @param pointOnCircle A point on the circle's circumference.
     * @return A Line object representing the tangent line.
     */
    public Line getTangent(Translation2d pointOnCircle) {
        return new Line(
                pointOnCircle.getX() - center.getX(),
                pointOnCircle.getY() - center.getY(),
                -center.getX() * (pointOnCircle.getX() - center.getX())
                        - center.getY() * (pointOnCircle.getY() - center.getY()) -
                        this.r * this.r
        );
    }

    /**
     * Calculates the tangent lines from a given external point to the circle.
     *
     * @param point A point outside or on the circle.
     * @return An array of Line objects representing the tangent lines.
     *         Returns an empty array if the point is inside the circle.
     */
    public Line[] getTangents(Translation2d point) {
        if (point.getDistance(this.center) < this.r) {
            return new Line[0];
        } else if (point.getDistance(this.center) == this.r) {
            return new Line[]{
                    getTangent(point)
            };
        }
        double centersDistance = this.center.getDistance(point);
        double newRad = Math.sqrt(Math.pow(centersDistance, 2) - Math.pow(this.r, 2));
        Circle newCircle = new Circle(point.getX(), point.getY(), newRad);
        Translation2d[] intersections = getInterSections(newCircle);
        Translation2d firstTanPoint = intersections[0];
        Translation2d secondTanPoint = intersections[1];

        return new Line[]{
                getTangent(firstTanPoint),
                getTangent(secondTanPoint)
        };
    }

    /**
     * Calculates the intersection points between this circle and another circle.
     *
     * @param other The other circle to find intersections with.
     * @return An array of Translation2d objects representing the intersection points.
     *         Returns an empty array if there are no intersections.
     */
    public Translation2d[] getInterSections(Circle other) {

        if (other.center.getDistance(this.center) > other.r + r) return new Translation2d[0];
        if (other.center.getDistance(this.center) < Math.abs(other.r - this.r)) return new Translation2d[0];

        if (other.center.getDistance(this.center) == other.r + r) {
            return new Translation2d[]{
                    this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle()))
            };
        }
        if (other.center.getDistance(this.center) < Math.abs(other.r - this.r)) {
            return new Translation2d[]{ // Check for edge case
                    this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle()))
            };
        }
        Rotation2d alpha = new Rotation2d(Math.acos(
                (Math.pow(other.r, 2) - Math.pow(this.r, 2) - Math.pow(this.center.getDistance(other.center), 2))
                        / (-2 * this.center.getDistance(other.center) * this.r)
        ));
        return new Translation2d[]{
                this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle().plus(alpha))),
                this.center.plus(new Translation2d(this.r, other.center.minus(this.center).getAngle().minus(alpha)))
        };
    }
}