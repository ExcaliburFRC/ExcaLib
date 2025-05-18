package frc.excalib.control.math;

import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicTask;

import java.util.function.DoubleSupplier;

public class Derivative extends PeriodicTask {
    private double value = 0.0;
    private double last;
    private final double periodSeconds;

    public Derivative(DoubleSupplier f, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.last = f.getAsDouble();
        this.periodSeconds = period.milliseconds / 1000.0;

        super.setTask(() -> {
            double current = f.getAsDouble();
            value = (current - last) / periodSeconds;
            last = current;
        });
    }

    public double getValue() {
        return value;
    }
}