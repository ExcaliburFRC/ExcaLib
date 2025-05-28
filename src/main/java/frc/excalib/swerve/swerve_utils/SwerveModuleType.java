package frc.excalib.swerve.swerve_utils;

import edu.wpi.first.math.util.Units;

/**
 * an enum class contains swerve types that helps to initialize the swerve
 */
public enum SwerveModuleType {
    MK4i_l3(Units.inchesToMeters(4) * Math.PI / 6.12,
            Units.inchesToMeters(4) * Math.PI / 6.12,
            (2 * Math.PI) / (21.4285714)
    );

    public final double kVelocityConversionFactor, kPositionConversionFactor, kSteeringVelocityConversionFactor;

    /**
     * @param velocityConversionFactor The velocity conversion factor of the module
     * @param positionConversionFactor The position conversion factor of the module
     * @param rotationVelocityConversionFactor The rotational velocity conversion factor of the module
     */
    SwerveModuleType(double velocityConversionFactor,
                     double positionConversionFactor,
                     double rotationVelocityConversionFactor) {
        kVelocityConversionFactor = velocityConversionFactor;
        kPositionConversionFactor = positionConversionFactor;
        kSteeringVelocityConversionFactor = rotationVelocityConversionFactor;
    }

    public double getVelocityConversionFactor() {
        return kVelocityConversionFactor;
    }

    public double getPositionConversionFactor() {
        return kPositionConversionFactor;
    }

    public double getSteeringVelocityConversionFactor() {
        return kSteeringVelocityConversionFactor;
    }
}
