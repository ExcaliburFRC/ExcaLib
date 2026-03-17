package frc.excalib.control.math;


import frc.excalib.control.math.periodics.PeriodicScheduler;
import frc.excalib.control.math.periodics.PeriodicTask;

import java.util.function.DoubleSupplier;

public class EMAFilter extends PeriodicTask {
    private final double alpha;
    private double filteredValue;
    private boolean initialized = false;

    public EMAFilter(DoubleSupplier input, double alpha, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.alpha = alpha;

        super.setTask(() -> {
            double raw = input.getAsDouble();
            if (!initialized) {
                filteredValue = raw;
                initialized = true;
            } else {
                filteredValue = alpha * raw + (1.0 - alpha) * filteredValue;
            }
        });
    }

    public double getValue() {
        return filteredValue;
    }
}