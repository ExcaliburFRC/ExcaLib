package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Translation2d;

import java.awt.*;

/**
 * Represents a line in a 2D plane, defined by the equation ax + by + c = 0.
 */
public class Line {
    // Coefficient of x in the line equation
    public final double a;
    // Coefficient of y in the line equation
    public final double b;
    // Constant term in the line equation
    public final double c;

    /**
     * Constructs a Line object with the specified coefficients.
     *
     * @param a The coefficient of x in the line equation.
     * @param b The coefficient of y in the line equation.
     * @param c The constant term in the line equation.
     */
    public Line(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /**
     * Constructs a Line object with two given points.
     *
     * @param pointA the first point
     * @param pointB the second point
     */
    public Line(Point pointA, Point pointB) {
        double MutA = pointB.getY() - pointA.getY();
        double MutB = pointA.getX() - pointB.getX();
        double MutC = pointB.getX() * pointA.getY() - pointA.getX() * pointB.getY();

        if (MutA < 0 || (MutA == 0 && MutB < 0)) {
            MutA *= -1;
            MutB *= -1;
            MutC *= -1;
        }

        this.a = MutA;
        this.b = MutB;
        this.c = MutC;
    }

    /**
     * Finds the intersection point of this line with another line.
     *
     * @param other The other line to find the intersection with.
     * @return A Translation2d object representing the intersection point.
     *         If the lines are parallel, the result may be undefined.
     */
    public Translation2d findIntersection(Line other) {
        return new Translation2d(
                (other.b * c - b * other.c) / (b * other.a - other.b * a),
                (other.a * c - a * other.c) / (a * other.b - other.a * b)
        );
    }

    /**
     * Returns a string representation of the line in the format "a: [a], b: [b], c: [c]".
     *
     * @return A string representation of the line.
     */
    @Override
    public String toString() {
        return "a: " + a + ", b: " + b + ", c: " + c;
    }
}