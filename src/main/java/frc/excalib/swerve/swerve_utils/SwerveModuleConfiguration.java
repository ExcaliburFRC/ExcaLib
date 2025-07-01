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
        //-----SDS-----
        // MK4
        MK4_L1(Units.inchesToMeters(4) * Math.PI / 8.14,
                Units.inchesToMeters(4) * Math.PI / 8.14,
                (2 * Math.PI) / 12.8),
        MK4_L2(Units.inchesToMeters(4) * Math.PI / 6.75,
                Units.inchesToMeters(4) * Math.PI / 6.75,
                (2 * Math.PI) / 12.8),
        MK4_L3(Units.inchesToMeters(4) * Math.PI / 6.12,
                Units.inchesToMeters(4) * Math.PI / 6.12,
                (2 * Math.PI) / 12.8),
        MK4_L4(Units.inchesToMeters(4) * Math.PI / 5.14,
                Units.inchesToMeters(4) * Math.PI / 5.14,
                (2 * Math.PI) / 12.8),

        // MK4i,
        MK4i_L1(Units.inchesToMeters(4) * Math.PI / 8.14,
                Units.inchesToMeters(4) * Math.PI / 8.14,
                (2 * Math.PI) / 21.4285714),
        MK4i_L2(Units.inchesToMeters(4) * Math.PI / 6.75,
                Units.inchesToMeters(4) * Math.PI / 6.75,
                (2 * Math.PI) / 21.4285714),
        MK4i_L3(Units.inchesToMeters(4) * Math.PI / 6.12,
                Units.inchesToMeters(4) * Math.PI / 6.12,
                (2 * Math.PI) / 21.4285714),
        MK4i_L4(Units.inchesToMeters(4) * Math.PI / 5.14,
                Units.inchesToMeters(4) * Math.PI / 5.14,
                (2 * Math.PI) / 21.4285714),

        //MK4c
        MK4c_L1(Units.inchesToMeters(4) * Math.PI / 7.13,
                Units.inchesToMeters(4) * Math.PI / 7.13,
                (2 * Math.PI) / 12.8),
        MK4c_L2(Units.inchesToMeters(4) * Math.PI / 5.9,
                Units.inchesToMeters(4) * Math.PI / 5.9,
                (2 * Math.PI) / 12.8),
        MK4c_L3(Units.inchesToMeters(4) * Math.PI / 5.36,
                Units.inchesToMeters(4) * Math.PI / 5.36,
                (2 * Math.PI) / 12.8),

        //MK4n
        MK4n_L1(Units.inchesToMeters(4) * Math.PI / 7.13,
                Units.inchesToMeters(4) * Math.PI / 7.13,
                (2 * Math.PI) / 18.75),
        MK4n_L2(Units.inchesToMeters(4) * Math.PI / 5.9,
                Units.inchesToMeters(4) * Math.PI / 5.9,
                (2 * Math.PI) / 18.75),
        MK4n_L3(Units.inchesToMeters(4) * Math.PI / 5.36,
                Units.inchesToMeters(4) * Math.PI / 5.36,
                (2 * Math.PI) / 18.75),

        //MK5n
        MK5n_R1(Units.inchesToMeters(4) * Math.PI / 7.03,
                Units.inchesToMeters(4) * Math.PI / 7.03,
                (2 * Math.PI) / 26.0909090909),
        MK5n_R2(Units.inchesToMeters(4) * Math.PI / 6.03,
                Units.inchesToMeters(4) * Math.PI / 6.03,
                (2 * Math.PI) / 26.0909090909),
        MK5n_R3(Units.inchesToMeters(4) * Math.PI / 5.27,
                Units.inchesToMeters(4) * Math.PI / 5.27,
                (2 * Math.PI) / 26.0909090909),

        //-----REV-----
        //MAXSwerve
        MAX_SWERVE_LOW(Units.inchesToMeters(3) * Math.PI / 5.50,
                Units.inchesToMeters(3) * Math.PI / 5.50,
                (2 * Math.PI) / (46.4236453202)),
        MAX_SWERVE_MEDIUM(Units.inchesToMeters(3) * Math.PI / 5.08,
                Units.inchesToMeters(3) * Math.PI / 5.08,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_HIGH(Units.inchesToMeters(3) * Math.PI / 5.08,
                Units.inchesToMeters(3) * Math.PI / 5.08,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_EXTRA_HIGH_1(Units.inchesToMeters(3) * Math.PI / 4.50,
                Units.inchesToMeters(3) * Math.PI / 4.50,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_EXTRA_HIGH_2(Units.inchesToMeters(3) * Math.PI / 4.29,
                Units.inchesToMeters(3) * Math.PI / 4.29,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_EXTRA_HIGH_3(Units.inchesToMeters(3) * Math.PI / 4.0,
                Units.inchesToMeters(3) * Math.PI / 4.0,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_EXTRA_HIGH_4(Units.inchesToMeters(3) * Math.PI / 3.75,
                Units.inchesToMeters(3) * Math.PI / 3.75,
                (2 * Math.PI) / 46.4236453202),
        MAX_SWERVE_EXTRA_HIGH_5(Units.inchesToMeters(3) * Math.PI / 3.56,
                Units.inchesToMeters(3) * Math.PI / 3.56,
                (2 * Math.PI) / 46.4236453202),

        //-----WCP-----
        //Swerve X
        SWERVE_X_X1_10T(Units.inchesToMeters(4) * Math.PI / 7.85,
                Units.inchesToMeters(4) * Math.PI / 7.85,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X1_11T(Units.inchesToMeters(4) * Math.PI / 7.13,
                Units.inchesToMeters(4) * Math.PI / 7.13,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X1_12T(Units.inchesToMeters(4) * Math.PI / 6.54,
                Units.inchesToMeters(4) * Math.PI / 6.54,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X2_10T(Units.inchesToMeters(4) * Math.PI / 6.56,
                Units.inchesToMeters(4) * Math.PI / 6.56,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X2_11T(Units.inchesToMeters(4) * Math.PI / 5.96,
                Units.inchesToMeters(4) * Math.PI / 5.96,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X2_12T(Units.inchesToMeters(4) * Math.PI / 5.46,
                Units.inchesToMeters(4) * Math.PI / 5.46,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X3_10T(Units.inchesToMeters(4) * Math.PI / 5.14,
                Units.inchesToMeters(4) * Math.PI / 5.14,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X3_11T(Units.inchesToMeters(4) * Math.PI / 4.75,
                Units.inchesToMeters(4) * Math.PI / 4.75,
                (2 * Math.PI) / 11.3142857143),
        SWERVE_X_X3_12T(Units.inchesToMeters(4) * Math.PI / 4.41,
                Units.inchesToMeters(4) * Math.PI / 4.41,
                (2 * Math.PI) / 11.3142857143),

        //Swerve XS
        SWERVE_XS_X1_12T(Units.inchesToMeters(3) * Math.PI / 6.0,
                Units.inchesToMeters(3) * Math.PI / 6.0,
                (2 * Math.PI) / 41.25),
        SWERVE_XS_X1_13T(Units.inchesToMeters(3) * Math.PI / 5.54,
                Units.inchesToMeters(3) * Math.PI / 5.54,
                (2 * Math.PI) / 41.25),
        SWERVE_XS_X1_14T(Units.inchesToMeters(3) * Math.PI / 5.14,
                Units.inchesToMeters(3) * Math.PI / 5.14,
                (2 * Math.PI) / 41.25),
        SWERVE_XS_X2_13T(Units.inchesToMeters(3) * Math.PI / 4.71,
                Units.inchesToMeters(3) * Math.PI / 4.71,
                (2 * Math.PI) / 41.25),
        SWERVE_XS_X2_15T(Units.inchesToMeters(3) * Math.PI / 4.40,
                Units.inchesToMeters(3) * Math.PI / 4.40,
                (2 * Math.PI) / 41.25),
        SWERVE_XS_X2_16T(Units.inchesToMeters(3) * Math.PI / 4.13,
                Units.inchesToMeters(3) * Math.PI / 4.13,
                (2 * Math.PI) / 41.25),

        //Swerve X Flipped
        SWERVE_X_FLIPPED_X1_10T(Units.inchesToMeters(4) * Math.PI / 8.10,
                Units.inchesToMeters(4) * Math.PI / 8.10,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X1_11T(Units.inchesToMeters(4) * Math.PI / 7.36,
                Units.inchesToMeters(4) * Math.PI / 7.36,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X1_12T(Units.inchesToMeters(4) * Math.PI / 6.75,
                Units.inchesToMeters(4) * Math.PI / 6.75,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X2_10T(Units.inchesToMeters(4) * Math.PI / 6.72,
                Units.inchesToMeters(4) * Math.PI / 6.72,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X2_11T(Units.inchesToMeters(4) * Math.PI / 6.11,
                Units.inchesToMeters(4) * Math.PI / 6.11,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X2_12T(Units.inchesToMeters(4) * Math.PI / 5.60,
                Units.inchesToMeters(4) * Math.PI / 5.60,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X3_10T(Units.inchesToMeters(4) * Math.PI / 5.51,
                Units.inchesToMeters(4) * Math.PI / 5.51,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X3_11T(Units.inchesToMeters(4) * Math.PI / 5.01,
                Units.inchesToMeters(4) * Math.PI / 5.01,
                (2 * Math.PI) / 13.3714285714),
        SWERVE_X_FLIPPED_X3_12T(Units.inchesToMeters(4) * Math.PI / 4.59,
                Units.inchesToMeters(4) * Math.PI / 4.59,
                (2 * Math.PI) / 13.3714285714),

        //Swerve X2, X2i, X2t
        SWERVE_X2_X1_10T(Units.inchesToMeters(4) * Math.PI / 7.67,
                Units.inchesToMeters(4) * Math.PI / 7.67,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X1_11T(Units.inchesToMeters(4) * Math.PI / 6.98,
                Units.inchesToMeters(4) * Math.PI / 6.98,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X1_12T(Units.inchesToMeters(4) * Math.PI / 6.39,
                Units.inchesToMeters(4) * Math.PI / 6.39,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X2_10T(Units.inchesToMeters(4) * Math.PI / 6.82,
                Units.inchesToMeters(4) * Math.PI / 6.82,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X2_11T(Units.inchesToMeters(4) * Math.PI / 6.20,
                Units.inchesToMeters(4) * Math.PI / 6.20,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X2_12T(Units.inchesToMeters(4) * Math.PI / 5.68,
                Units.inchesToMeters(4) * Math.PI / 5.68,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X3_10T(Units.inchesToMeters(4) * Math.PI / 6.48,
                Units.inchesToMeters(4) * Math.PI / 6.48,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X3_11T(Units.inchesToMeters(4) * Math.PI / 5.89,
                Units.inchesToMeters(4) * Math.PI / 5.89,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X3_12T(Units.inchesToMeters(4) * Math.PI / 5.40,
                Units.inchesToMeters(4) * Math.PI / 5.40,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X4_10T(Units.inchesToMeters(4) * Math.PI / 5.67,
                Units.inchesToMeters(4) * Math.PI / 5.67,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X4_11T(Units.inchesToMeters(4) * Math.PI / 5.15,
                Units.inchesToMeters(4) * Math.PI / 5.15,
                (2 * Math.PI) / 12.1),
        SWERVE_X2_X4_12T(Units.inchesToMeters(4) * Math.PI / 4.73,
                Units.inchesToMeters(4) * Math.PI / 4.73,
                (2 * Math.PI) / 12.1),

        //Swerve X2S, X2St
        SWERVE_X2S_X1_15T(Units.inchesToMeters(3.5) * Math.PI / 6.0,
                Units.inchesToMeters(3.5) * Math.PI / 6.0,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X1_16T(Units.inchesToMeters(3.5) * Math.PI / 5.63,
                Units.inchesToMeters(3.5) * Math.PI / 5.63,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X1_17T(Units.inchesToMeters(3.5) * Math.PI / 5.29,
                Units.inchesToMeters(3.5) * Math.PI / 5.29,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X2_17T(Units.inchesToMeters(3.5) * Math.PI / 4.94,
                Units.inchesToMeters(3.5) * Math.PI / 4.94,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X2_18T(Units.inchesToMeters(3.5) * Math.PI / 4.67,
                Units.inchesToMeters(3.5) * Math.PI / 4.67,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X2_19T(Units.inchesToMeters(3.5) * Math.PI / 4.42,
                Units.inchesToMeters(3.5) * Math.PI / 4.42,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X3_19T(Units.inchesToMeters(3.5) * Math.PI / 4.11,
                Units.inchesToMeters(3.5) * Math.PI / 4.11,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X3_20T(Units.inchesToMeters(3.5) * Math.PI / 3.90,
                Units.inchesToMeters(3.5) * Math.PI / 3.90,
                (2 * Math.PI) / 25.9),
        SWERVE_X2S_X3_21T(Units.inchesToMeters(3.5) * Math.PI / 3.71,
                Units.inchesToMeters(3.5) * Math.PI / 3.71,
                (2 * Math.PI) / 25.9);

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

        public SwerveModuleConfiguration getModuleConfiguration() {
            return m_moduleConfiguration;
        }
    }
}
