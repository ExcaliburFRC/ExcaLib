package frc.excalib.control.math;

import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicTask;

import java.util.function.DoubleSupplier;

public class Integral extends PeriodicTask {
    private double value;
    private double last;

    public Integral(double initialValue, DoubleSupplier f, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.last = f.getAsDouble();
        this.value = initialValue;

        double periodSeconds = period.milliseconds / 1000.0;

        super.setTask(() -> {
            double current = f.getAsDouble();
            value += periodSeconds * (current + last) / 2.0;
            last = current;
        });
    }

    public double getValue() {
        return value;
    }
}
