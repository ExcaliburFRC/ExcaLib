package frc.excalib.control.math.filters.data_fusion;

import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicTask;

import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

public class WMAFilter extends PeriodicTask {
    public final int windowSize;
    private final Deque<Double> window;
    private final IntFunction<Double> weightFunction;

    public WMAFilter(DoubleSupplier input, int windowSize, IntFunction<Double> weightFunction, PeriodicScheduler.PERIOD period) {
        super(() -> {}, period);
        this.windowSize = windowSize;
        this.weightFunction = weightFunction;
        this.window = new ArrayDeque<>(windowSize);

        super.setTask(() -> {
            double sample = input.getAsDouble();
            if (window.size() == windowSize) {
                window.removeFirst();
            }
            window.addLast(sample);
        });
    }

    public double getValue() {
        if (window.isEmpty()) return 0;

        double weightedSum = 0;
        double weightTotal = 0;

        Iterator<Double> iter = window.iterator();
        int index = 0;
        while (iter.hasNext()) {
            double sample = iter.next();
            double weight = weightFunction.apply(index);
            weightedSum += sample * weight;
            weightTotal += weight;
            index++;
        }

        return weightTotal == 0 ? 0 : weightedSum / weightTotal;
    }
}
