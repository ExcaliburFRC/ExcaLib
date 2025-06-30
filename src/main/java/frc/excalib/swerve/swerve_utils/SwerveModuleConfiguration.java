package frc.excalib.swerve.swerve_utils;

import edu.wpi.first.math.util.Units;

/**
 * A record represents the module configuration
 *
 * @param velocityConversionFactor         The velocity conversion factor of the module
 * @param positionConversionFactor         The position conversion factor of the module
 * @param steeringVelocityConversionFactor The rotational velocity conversion factor of the module
 */
public record SwerveModuleConfiguration(double velocityConversionFactor,
                                        double positionConversionFactor,
                                        double steeringVelocityConversionFactor) {

    /**
     * an enum class contains swerve types that helps to initialize the swerve
     */
    public enum SwerveModuleType {
        MK4i_l3(Units.inchesToMeters(4) * Math.PI / 6.12,
                Units.inchesToMeters(4) * Math.PI / 6.12,
                (2 * Math.PI) / (21.4285714));

        public final SwerveModuleConfiguration m_moduleConfiguration;

        /**
         * @param velocityConversionFactor         The velocity conversion factor of the module
         * @param positionConversionFactor         The position conversion factor of the module
         * @param steeringVelocityConversionFactor The rotational velocity conversion factor of the module
         */
        SwerveModuleType(double velocityConversionFactor,
                         double positionConversionFactor,
                         double steeringVelocityConversionFactor) {
            m_moduleConfiguration = new SwerveModuleConfiguration(velocityConversionFactor, positionConversionFactor, steeringVelocityConversionFactor);
        }

        public double getVelocityConversionFactor() {
            return m_moduleConfiguration.velocityConversionFactor;
        }

        public double getPositionConversionFactor() {
            return m_moduleConfiguration.positionConversionFactor;
        }

        public double getSteeringVelocityConversionFactor() {
            return m_moduleConfiguration.steeringVelocityConversionFactor;
        }

        public SwerveModuleConfiguration getModuleConfiguration() {
            return m_moduleConfiguration;
        }
    }
}
