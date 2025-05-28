package frc.excalib.swerve.swerve_utils;

import java.util.function.DoubleSupplier;

/**
 * A record that includes the swerve specifications
 *
 * @param maxVelocity       the max linear velocity of the swerve
 * @param maxOmegaRadPerSec the max angular velocity of the swerve
 * @param maxAcc            the max acceleration of the swerve
 * @param maxFrontAcc       the max front acceleration of the swerve as a DoubleSupplier
 * @param maxSideAcc        the max side acceleration of the swerve as a DoubleSupplier
 * @param maxForwardAcc     the max forward acceleration of the swerve
 */
public record SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc,
                          DoubleSupplier maxFrontAcc, DoubleSupplier maxSideAcc, double maxForwardAcc) {
    /**
     * A constructor for robots without changing height (ex: without elevator etc.)
     *
     * @param maxVelocity       the max linear velocity of the swerve
     * @param maxOmegaRadPerSec the max angular velocity of the swerve
     * @param maxAcc            the max acceleration of the swerve
     * @param maxFrontAcc       the max front acceleration of the swerve
     * @param maxSideAcc        the max side acceleration of the swerve
     * @param maxForwardAcc     the max forward acceleration of the swerve
     */
    public SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc,
                       double maxFrontAcc, double maxSideAcc, double maxForwardAcc) {
        this(maxVelocity, maxOmegaRadPerSec, maxAcc, () -> maxFrontAcc, () -> maxSideAcc, maxForwardAcc);
    }

    /**
     * A constructor for basic specifications
     *
     * @param maxVelocity       the max linear velocity of the swerve
     * @param maxOmegaRadPerSec the max angular velocity of the swerve
     * @param maxAcc            the max acceleration of the swerve
     */
    public SwerveSpecs(double maxVelocity, double maxOmegaRadPerSec, double maxAcc) {
        this(maxVelocity, maxOmegaRadPerSec, maxAcc, () -> 10, () -> 10, 10);
    }
}
