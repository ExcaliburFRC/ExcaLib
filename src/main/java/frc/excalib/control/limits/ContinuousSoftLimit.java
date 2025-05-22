package frc.excalib.control.limits;

import edu.wpi.first.math.Pair;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.math.MathUtils.continuousLimit;

/**
 * A class representing the allowed one dimensional range for the state of a system with continuous input (ex: turret).
 * the range is defined by two dynamic limits
 */
public class ContinuousSoftLimit extends SoftLimit {
    /**
     * A constructor that takes two DoubleSuppliers representing the dynamic limits:
     *
     * @param minLimit the minimal limit of the represented range
     * @param maxLimit the maximal limit of the represented range
     */
    public ContinuousSoftLimit(DoubleSupplier minLimit, DoubleSupplier maxLimit) {
        super(minLimit, maxLimit);
    }

    /**
     * A method to get the optimized setpoint.
     *
     * @param measurement    the current state of the system
     * @param wantedSetpoint the wanted state of the system
     * @return the optimized setpoint
     */
    public double getOptimizedSetpoint(double measurement, double wantedSetpoint) {
        Optional<Pair<Double, Double>> setpoints = continuousLimit(wantedSetpoint, measurement);
        if (setpoints.isEmpty()) { // the measurement and the setpoint are equals
            return wantedSetpoint;
        }

        double upperSetpoint, lowerSetpoint;
        upperSetpoint = setpoints.get().getFirst();
        lowerSetpoint = setpoints.get().getSecond();

        // if one of the setpoints exceeds the limit, return the other
        if (upperSetpoint > super.getMaxLimit()) {
            return lowerSetpoint;
        } else if (lowerSetpoint < super.getMinLimit()) {
            return upperSetpoint;
        }
        return Math.abs(measurement - upperSetpoint) < Math.abs(measurement - lowerSetpoint) ?
                upperSetpoint :
                lowerSetpoint;
    }

    /**
     * A method to get the unoptimized setpoint.
     *
     * @param measurement    the current state of the system
     * @param wantedSetpoint the wanted state of the system
     * @return the unoptimized setpoint
     */
    public double getUnoptimizedSetpoint(double measurement, double wantedSetpoint) {
        Optional<Pair<Double, Double>> setpoints = continuousLimit(wantedSetpoint, measurement);
        if (setpoints.isEmpty()) { // the measurement and the setpoint are equals
            return wantedSetpoint;
        }

        double upperSetpoint, lowerSetpoint;
        upperSetpoint = setpoints.get().getFirst();
        lowerSetpoint = setpoints.get().getSecond();

        // if one of the setpoints exceeds the limit, return the other
        if (upperSetpoint > super.getMaxLimit()) {
            return lowerSetpoint;
        } else if (lowerSetpoint < super.getMinLimit()) {
            return upperSetpoint;
        }

        return Math.abs(measurement - upperSetpoint) < Math.abs(measurement - lowerSetpoint) ?
                lowerSetpoint :
                upperSetpoint;
    }
}
