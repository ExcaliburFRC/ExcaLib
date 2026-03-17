package frc.excalib.control.math.physics;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;

/**
 * This Class represents the mass of an object including size, center
 * and utility functions.
 */
public class Mass {
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final double mass;

    /**
     * returns a new Mass object
     * @param xSupplier the x position of the Mass (m)
     * @param ySupplier the y position of the Mass (m)
     * @param mass the size value of the mass (kg)
     */
    public Mass(DoubleSupplier xSupplier, DoubleSupplier ySupplier, double mass) {
        this.m_xSupplier = xSupplier;
        this.m_ySupplier = ySupplier;
        this.mass = mass;
    }

    /**
     * @return the center of the mass as a Translation2d
     */
    public Translation2d getCenterOfMass() {
        return new Translation2d(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());
    }

    /**
     * @param other the mass to add
     * @return a new Mass representing the mass of the system the includes both masses.
     */
    public Mass add(Mass other) {
        return new Mass(
                () ->
                        (other.mass * other.m_xSupplier.getAsDouble() +
                                this.mass * this.m_xSupplier.getAsDouble()) / (other.mass + this.mass),
                () ->
                        (other.mass * other.m_ySupplier.getAsDouble() +
                                this.mass * this.m_ySupplier.getAsDouble()) / (other.mass + this.mass),
                other.mass + this.mass
        );
    }
}