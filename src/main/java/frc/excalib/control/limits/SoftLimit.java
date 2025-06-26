package frc.excalib.control.limits;


import edu.wpi.first.epilogue.Logged;

import java.util.function.DoubleSupplier;

/**
 * A class representing the allowed one dimensional range for the state of a system.
 * the range is defined by two dynamic limits
 */
@Logged
public class SoftLimit {
    private final DoubleSupplier m_minLimit, m_maxLimit;

    /**
     * A constructor that takes two DoubleSuppliers representing the dynamic limits:
     * @param minLimit the minimal limit of the represented range
     * @param maxLimit the maximal limit of the represented range
     */
    public SoftLimit(DoubleSupplier minLimit, DoubleSupplier maxLimit) {
        m_minLimit = minLimit;
        m_maxLimit = maxLimit;
    }

    /**
     * Check if the value is within the limits
     * @param val the value to check
     * @return if the value is in the limit or not
     */
    @Logged
    public boolean within(double val) {
        return (m_maxLimit.getAsDouble() >= val) && (val >= m_minLimit.getAsDouble());
    }

    /**
     * A method to limit a value to the represented range
     * @param val the value to limit
     * @return the limited value
     */
    @Logged
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
    @Logged
    public double getMaxLimit() {
        return m_maxLimit.getAsDouble();
    }

    /**
     * @return the current minimal limit of the represented range
     */
    @Logged
    public double getMinLimit() {
        return m_minLimit.getAsDouble();
    }
}
