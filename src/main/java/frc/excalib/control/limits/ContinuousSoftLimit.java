package frc.excalib.control.limits;

import java.util.function.DoubleSupplier;

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
     * A method to get the limited setpoint.
     *
     * @param measurement the current state of the system
     * @param wantedSetpoint the wanted state of the system
     * @return the limited setpoint
     */
    public double getSetpoint(double measurement, double wantedSetpoint) {
        double upperSetpoint, lowerSetpoint;
        if (wantedSetpoint > measurement) {
            upperSetpoint = wantedSetpoint;
            while ((upperSetpoint - 2 * Math.PI) > measurement) {
                upperSetpoint -= 2 * Math.PI;
            }
            lowerSetpoint = upperSetpoint - 2 * Math.PI;
        } else if (wantedSetpoint < measurement) {
            lowerSetpoint = wantedSetpoint;
            while ((lowerSetpoint + 2 * Math.PI) < measurement) {
                lowerSetpoint += 2 * Math.PI;
            }
            upperSetpoint = lowerSetpoint + 2 * Math.PI;
        } else {
            return wantedSetpoint;
        }
        if (upperSetpoint > super.getMaxLimit()) {
            return lowerSetpoint;
        } else if (lowerSetpoint < super.getMinLimit()) {
            return upperSetpoint;
        }
        return
                Math.abs(measurement - upperSetpoint) < Math.abs(measurement - lowerSetpoint) ?
                        upperSetpoint : lowerSetpoint;
    }
}
