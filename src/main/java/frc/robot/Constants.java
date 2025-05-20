package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;

public class Constants {
    public static final class CannonConstants{

        // turret constants
        public static final double UPPER_LIMIT = 450;
        public static final double LOWER_LIMIT = 0;

        public static final double TURRET_POSITION_CONVERSION_FACTOR = 1.0 / 150.0 * 360;
        public static final double TURRET_VELOCITY_CONVERSION_FACTOR = 1.0 / 150.0 * 360;

        public static final ContinuousSoftLimit TURRET_SOFT_LIMIT = new ContinuousSoftLimit(()-> UPPER_LIMIT, ()-> LOWER_LIMIT);
        public static final Gains TURRET_GAINS = new Gains(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7);
        public static final TrapezoidProfile.Constraints TURRET_CONSTRAINTS = new TrapezoidProfile.Constraints(6.28, 3.14);

        public static final double TURRET_TOLERANCE = 3; // deg

        // arm constants
        public static final Gains ARM_GAINS = new Gains(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7);
        public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS = new TrapezoidProfile.Constraints(6.28, 3.14);

        public static final double ARM_POSITION_CONVERSION_FACTOR = 1.0 / 452.0 * 360;
        public static final double ENCODER_OFFSET = 0.1234;

        public static final double ARM_TOLERANCE = 3; // deg
    }
}
