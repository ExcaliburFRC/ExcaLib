package frc.excalib.control.math.filters.data_fusion;

import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicTask;

import java.util.function.DoubleSupplier;
import java.util.ArrayDeque;
import java.util.Deque;

public class SMAFilter extends PeriodicTask {
    private final int windowSize;
    private final Deque<Double> window;
    private double sum;

    public SMAFilter(DoubleSupplier input, int windowSize, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.windowSize = windowSize;
        this.window = new ArrayDeque<>(windowSize);
        this.sum = 0;

        super.setTask(() -> {
            double sample = input.getAsDouble();
            if (window.size() == windowSize) {
                sum -= window.removeFirst();
            }
            window.addLast(sample);
            sum += sample;
        });
    }

    public double getValue() {
        if (window.isEmpty()) return 0;
        return sum / window.size();
    }
}
