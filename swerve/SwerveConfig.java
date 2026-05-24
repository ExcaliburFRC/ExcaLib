package frc.excalib.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import frc.excalib.control.gains.Gains;

/**
 * Configuration injection container for the generic Swerve drive system.
 * Removes direct library dependency on robot-specific Constants classes.
 */
public class SwerveConfig {
    public Gains translationGains;
    public Gains angleGains;
    public PIDConstants translationPPConstants;
    public PIDConstants anglePPConstants;
    public PathConstraints maxPathConstraints;
    public double maxVel;

    /**
     * Empty constructor for builder-style configuration.
     */
    public SwerveConfig() {}

    /**
     * Complete parameter constructor.
     */
    public SwerveConfig(
            Gains translationGains,
            Gains angleGains,
            PIDConstants translationPPConstants,
            PIDConstants anglePPConstants,
            PathConstraints maxPathConstraints,
            double maxVel) {
        this.translationGains = translationGains;
        this.angleGains = angleGains;
        this.translationPPConstants = translationPPConstants;
        this.anglePPConstants = anglePPConstants;
        this.maxPathConstraints = maxPathConstraints;
        this.maxVel = maxVel;
    }
}
