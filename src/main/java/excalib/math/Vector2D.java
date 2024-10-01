package excalib.math;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class representing a Vector in two dimensions;
 * the Vector is defined by x value and y value
 *
 * @author Itay Keller
 */
public class Vector2D {
    private final double m_x, m_y;

    /**
     * A constructor that takes two doubles representing the x and y components:
     *
     * @param x The x component of the Vector
     * @param y The y component of the Vector
     */
    public Vector2D(double x, double y) {
        this.m_x = x;
        this.m_y = y;
    }

    /**
     * A constructor that takes a double representing the distance,
     * and a Rotation2D representing the angle:
     *
     * @param magnitude The distance from the origin
     * @param direction The angle of the Vector
     */
    public Vector2D(double magnitude, Rotation2d direction) {
        this.m_x = magnitude * direction.getCos();
        this.m_y = magnitude * direction.getSin();
    }

    /**
     * A function to get the x component of the Vector
     *
     * @return x component
     */
    public double getX() {
        return m_x;
    }

    /**
     * A function to get the y component of the Vector
     *
     * @return y component
     */
    public double getY() {
        return m_y;
    }

    /**
     * A function to get the distance from the origin
     *
     * @return distance
     */
    public double getDistance() {
        return Math.sqrt(m_x * m_x + m_y * m_y);
    }

    /**
     * A function to get the direction of the Vector
     *
     * @return direction
     */
    public Rotation2d getDirection() {
        return new Rotation2d(Math.atan(m_y / m_x));
    }

    /**
     * A function that adds other Vector to this Vector
     *
     * @return a new Vector2D that represents the sum of the Vectors
     */
    public Vector2D plus(Vector2D other) {
        return new Vector2D(m_x + other.m_x, m_y + other.m_y);
    }

    /**
     * A function that multiplies the Vector by a scalar
     *
     * @return a new Vector2D that represents the product of the Vectors
     */
    public Vector2D mul(double scalar) {
        return new Vector2D(m_x * scalar, m_y * scalar);
    }

    /**
     * A function that rotates the Vector by a direction
     *
     * @return a new Vector2D that represents the rotated Vector
     */
    public Vector2D rotate(Rotation2d deltaDirection) {
        return new Vector2D(getDistance(), getDirection().rotateBy(deltaDirection));
    }
}
