package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class representing a Vector in two dimensions;
 * the Vector is defined by x value and y value
 *
 * @author Itay
 */
public class Vector2D {
    private double m_x, m_y;

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
        return Math.hypot(m_x, m_y);
    }

    /**
     * A function to get the direction of the Vector
     *
     * @return direction
     */
    public Rotation2d getDirection() {
        return new Rotation2d(Math.atan2(m_y, m_x));
    }

    /**
     * A function that adds another Vector to this Vector
     *
     * @param other The other vector to add
     * @return a new Vector2D that represents the sum of the Vectors
     */
    public Vector2D plus(Vector2D other) {
        return new Vector2D(m_x + other.m_x, m_y + other.m_y);
    }

    /**
     * A function that multiplies the Vector by a scalar
     *
     * @param scalar The scalar to multiply by
     * @return a new Vector2D that represents the scaled Vector
     */
    public Vector2D mul(double scalar) {
        return new Vector2D(m_x * scalar, m_y * scalar);
    }

    /**
     * A function that rotates the Vector by a given angle
     *
     * @param deltaDirection The angle to rotate by
     * @return a new Vector2D that represents the rotated Vector
     */
    public Vector2D rotate(Rotation2d deltaDirection) {
        double cosTheta = deltaDirection.getCos();
        double sinTheta = deltaDirection.getSin();

        double newX = m_x * cosTheta - m_y * sinTheta;
        double newY = m_x * sinTheta + m_y * cosTheta;

        return new Vector2D(newX, newY);
    }

    /**
     * Sets the x component of the Vector
     *
     * @param x The new x component
     */
    public void setX(double x) {
        this.m_x = x;
    }

    /**
     * Sets the y component of the Vector
     *
     * @param y The new y component
     */
    public void setY(double y) {
        this.m_y = y;
    }

    /**
     * Sets the magnitude of the Vector while keeping its direction
     *
     * @param magnitude The new magnitude
     */
    public void setMagnitude(double magnitude) {
        double currentMagnitude = getDistance();
        if (currentMagnitude != 0) {
            double scale = magnitude / currentMagnitude;
            this.m_x *= scale;
            this.m_y *= scale;
        } else {
            // When the current magnitude is zero, direction is undefined.
            // Default to setting x to the magnitude and y to zero.
            this.m_x = magnitude;
            this.m_y = 0;
        }
    }

    /**
     * Sets the direction of the Vector while keeping its magnitude
     *
     * @param direction The new direction
     */
    public void setDirection(Rotation2d direction) {
        double magnitude = getDistance();
        this.m_x = magnitude * direction.getCos();
        this.m_y = magnitude * direction.getSin();
    }

    public Vector2D limit(Vector2D limit) {
        Vector2D output = new Vector2D(m_x, m_y);
        output.rotate(limit.getDirection().unaryMinus());
        output.setX(MathUtils.limitTo(limit.getDistance(), output.m_x));
        output.setDirection(this.getDirection());
        return output;
    }
}