package frc.excalib.control.limits;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil; // Import the MathUtil class

public class ContinuousSoftLimit extends SoftLimit {
    private static final double TWO_PI = 2 * Math.PI;

    public ContinuousSoftLimit(DoubleSupplier minLimit, DoubleSupplier maxLimit) {
        super(minLimit, maxLimit);
    }

    public double getSetpoint(double measurement, double wantedSetPoint) {
        double minLimit = super.getMinLimit();
        double maxLimit = super.getMaxLimit();

        // Calculate the difference and normalize it to (-PI, PI]
        // This ensures 'candidate1' is the shortest angular path from 'measurement'
        double angleDifference = MathUtil.angleModulus(wantedSetPoint - measurement);

        // Candidate 1: The target reached by the shortest angular path
        double candidate1 = measurement + angleDifference;

        // Candidate 2: The target reached by taking the "other way around"
        // If angleDifference was positive (clockwise), candidate2 is counter-clockwise.
        // If angleDifference was negative (counter-clockwise), candidate2 is clockwise.
        double candidate2 = (angleDifference > 0) ? (candidate1 - TWO_PI) : (candidate1 + TWO_PI);

        // Check if candidates are within the soft limits
        boolean candidate1Valid = candidate1 >= minLimit && candidate1 <= maxLimit;
        boolean candidate2Valid = candidate2 >= minLimit && candidate2 <= maxLimit;

        if (candidate1Valid && candidate2Valid)
            // Both paths lead to a valid angle, choose the one truly closer to the measurement
            return Math.abs(measurement - candidate1) < Math.abs(measurement - candidate2) ? candidate1 : candidate2;
        if (candidate1Valid)
            // Only candidate1 is valid
            return candidate1;
        if (candidate2Valid)
            // Only candidate2 is valid
            return candidate2;

            // Neither of the primary cyclic candidates are valid.
            // This means the 'wantedSetPoint' is unreachable given the current `measurement`
            // and the `minLimit`/`maxLimit` constraints.
            // In this case, the most robust action is to clamp the desired setpoint
            // to the absolute bounds of the soft limit. This ensures the returned value
            // is always within the safe range.
        return super.limit(wantedSetPoint);
        }
    }
}