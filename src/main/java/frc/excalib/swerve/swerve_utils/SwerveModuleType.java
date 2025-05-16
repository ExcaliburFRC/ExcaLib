package frc.excalib.swerve.swerve_utils;

import edu.wpi.first.math.util.Units;

public enum SwerveModuleType {
    MK4i_l3(Units.inchesToMeters(4) * Math.PI / 6.12,
            Units.inchesToMeters(4) * Math.PI / 6.12,
            (2 * Math.PI) / (21.4285714)
    );

    public final double kVelocityConversionFactor, kPositionConversionFactor, kSteeringVelocityConversionFactor;

    SwerveModuleType(double velocityConversionFactor,
                     double positionConversionFactor,
                     double rotationVelocityConversionFactor) {
        kVelocityConversionFactor = velocityConversionFactor;
        kPositionConversionFactor = positionConversionFactor;
        kSteeringVelocityConversionFactor = rotationVelocityConversionFactor;
    }
}
