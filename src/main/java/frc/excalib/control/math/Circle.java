package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a circle in a 2D plane, defined by its center and radius.
 */
public class Circle extends Ellipse2d {
    /**
     * Constructs a Circle object with the specified center coordinates and radius.
     *
     * @param a The x-coordinate of the circle's center.
     * @param b The y-coordinate of the circle's center.
     * @param r The radius of the circle.
     */
    public Circle(double a, double b, double r) {
        super(new Translation2d(a, b), r);
    }

    /**
     * Calculates the tangent line to the circle at a given point on its circumference.
     *
     * @param pointOnCircle A point on the circle's circumference.
     * @return A Line object representing the tangent line.
     */
    public Line getTangent(Translation2d pointOnCircle) {
        return new Line(
                pointOnCircle.getX() - super.getCenter().getX(),
                pointOnCircle.getY() - super.getCenter().getY(),
                -super.getCenter().getX() * (pointOnCircle.getX() - super.getCenter().getX())
                        - super.getCenter().getY() * (pointOnCircle.getY() - super.getCenter().getY()) -
                        super.getXSemiAxis() * super.getXSemiAxis()
        );
    }

    /**
     * Calculates the tangent lines from a given external point to the circle.
     *
     * @param point A point outside or on the circle.
     * @return An array of Line objects representing the tangent lines.
     * Returns an empty array if the point is inside the circle.
     */
    public Line[] getTangents(Translation2d point) {
        if (point.getDistance(super.getCenter().getTranslation()) < super.getXSemiAxis()) {
            return new Line[0];
        } else if (point.getDistance(super.getCenter().getTranslation()) == super.getXSemiAxis()) {
            return new Line[]{
                    getTangent(point)
            };
        }
        double centersDistance = super.getCenter().getTranslation().getDistance(point);
        double newRad = Math.sqrt(Math.pow(centersDistance, 2) - Math.pow(super.getXSemiAxis(), 2));
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
     * Constructs a Line that passes through the given point and the center of the circle.
     *
     * @param point The point from which the line to the circle's center is drawn.
     * @return A Line object representing the line passing through the given point and the circle's center.
     */
    public Line getLineToCenter(Translation2d point) {
        Translation2d center = super.getCenter().getTranslation();
        return new Line(
                point.getY() - center.getY(),
                center.getX() - point.getX(),
                point.getX() * center.getY() - center.getX() * point.getY()
        );
    }

    /**
     * Calculates the intersection points between this circle and a line.
     *
     * @param line The line to intersect with.
     * @return An array of Translation2d objects representing the intersection points.
     *         Returns an empty array if there are no intersections.
     */
    public Translation2d[] getIntersections(Line line) {
        double h = super.getCenter().getX();
        double k = super.getCenter().getY();
        double r = super.getXSemiAxis();
        double a = line.a;
        double b = line.b;
        double c = line.c;

        if (b == 0) {
            // Line: x = -c / a
            double x = -c / a;
            double dx = x - h;
            double underSqrt = r * r - dx * dx;
            if (underSqrt < 0) return new Translation2d[0];
            else if (underSqrt == 0) {
                return new Translation2d[]{ new Translation2d(x, k) };
            } else {
                double sqrtVal = Math.sqrt(underSqrt);
                double y1 = k + sqrtVal;
                double y2 = k - sqrtVal;
                return new Translation2d[]{
                        new Translation2d(x, y1),
                        new Translation2d(x, y2)
                };
            }
        }

        // General case: express y in terms of x from line equation
        // y = (-a*x - c) / b
        // Plug into circle: (x - h)^2 + (y - k)^2 = r^2
        double m = -a / b;
        double yInt = -c / b;

        // (x - h)^2 + (m*x + yInt - k)^2 = r^2
        double A = 1 + m * m;
        double B = 2 * (m * (yInt - k) - h);
        double C = h * h + (yInt - k) * (yInt - k) - r * r;

        double discriminant = B * B - 4 * A * C;

        if (discriminant < 0) {
            return new Translation2d[0];
        } else if (discriminant == 0) {
            double x = -B / (2 * A);
            double y = m * x + yInt;
            return new Translation2d[]{ new Translation2d(x, y) };
        } else {
            double sqrtDisc = Math.sqrt(discriminant);
            double x1 = (-B + sqrtDisc) / (2 * A);
            double y1 = m * x1 + yInt;
            double x2 = (-B - sqrtDisc) / (2 * A);
            double y2 = m * x2 + yInt;
            return new Translation2d[]{
                    new Translation2d(x1, y1),
                    new Translation2d(x2, y2)
            };
        }
    }


    /**
     * Calculates the intersection points between this circle and another circle.
     *
     * @param other The other circle to find intersections with.
     * @return An array of Translation2d objects representing the intersection points.
     * Returns an empty array if there are no intersections.
     */
    public Translation2d[] getInterSections(Circle other) {
        Translation2d thisCenter = super.getCenter().getTranslation();
        Translation2d otherCenter = other.getCenter().getTranslation();

        if (otherCenter.getDistance(thisCenter) > other.getXSemiAxis() + super.getXSemiAxis())
            return new Translation2d[0];
        if (otherCenter.getDistance(thisCenter) < Math.abs(other.getXSemiAxis() - super.getXSemiAxis()))
            return new Translation2d[0];

        if (otherCenter.getDistance(thisCenter) == other.getXSemiAxis() + super.getXSemiAxis()) {
            return new Translation2d[]{
                    thisCenter.plus(new Translation2d(super.getXSemiAxis(), otherCenter.minus(thisCenter).getAngle()))
            };
        }
        if (otherCenter.getDistance(thisCenter) < Math.abs(other.getXSemiAxis() - super.getXSemiAxis())) {
            return new Translation2d[]{ // Check for edge case
                    thisCenter.plus(new Translation2d(super.getXSemiAxis(), otherCenter.minus(thisCenter).getAngle()))
            };
        }
        Rotation2d alpha = new Rotation2d(Math.acos(
                (Math.pow(other.getXSemiAxis(), 2) - Math.pow(super.getXSemiAxis(), 2) - Math.pow(thisCenter.getDistance(otherCenter), 2))
                        / (-2 * thisCenter.getDistance(otherCenter) * super.getXSemiAxis())
        ));
        return new Translation2d[]{
                thisCenter.plus(new Translation2d(super.getXSemiAxis(), otherCenter.minus(thisCenter).getAngle().plus(alpha))),
                thisCenter.plus(new Translation2d(super.getXSemiAxis(), otherCenter.minus(thisCenter).getAngle().minus(alpha)))
        };
    }
}