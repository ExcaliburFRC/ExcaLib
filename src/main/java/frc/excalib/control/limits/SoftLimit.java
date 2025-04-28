package frc.excalib.control.limits;

import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

/**
 * A class representing the allowed one dimensional range for the state of a system.
 * the range is defined by two dynamic limits
 *
 * @author Yoav Cohen
 */
public class SoftLimit implements Logged {
    private final DoubleSupplier m_minLimit, m_maxLimit;

    /**
     * A constructor that takes two DoubleSuppliers representing the dynamic limits:
     *
     * @param minLimit the minimal limit of the represented range
     * @param maxLimit the maximal limit of the represented range
     */
    public SoftLimit(DoubleSupplier minLimit, DoubleSupplier maxLimit) {
        m_minLimit = minLimit;
        m_maxLimit = maxLimit;
    }

    /**
     * Check if the value is within the limits
     *
     * @param val the value to check
     * @return if it is in the limit or not
     */
    @Log.NT
    public boolean within(double val) {
        return (m_maxLimit.getAsDouble() >= val) && (val >= m_minLimit.getAsDouble());
    }

    /**
     * A method to limit a value to the represented range
     *
     * @param val the value to limit
     * @return the limited value
     */
    @Log.NT
    public double limit(double val) {
        if (within(val)) {
            return val;
        }
        if (val > m_maxLimit.getAsDouble()) {
            return m_maxLimit.getAsDouble();
        }
        return m_minLimit.getAsDouble();
    }

    /**
     * @return the current maximal limit of the represented range
     */
    @Log.NT
    public double getMaxLimit() {
        return m_maxLimit.getAsDouble();
    }

    /**
     * @return the current minimal limit of the represented range
     */
    @Log.NT
    public double getMinLimit() {
        return m_minLimit.getAsDouble();
    }
}
