// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

/**
 * Tracks and analyzes robot performance metrics throughout a match.
 * Helps identify bottlenecks and optimization opportunities.
 */
public class PerformanceMetricsTracker {

    private static final int MAX_HISTORY_SIZE = 300; // ~6 seconds of data at 50Hz

    private final List<Long> commandExecutionTimes = new ArrayList<>();
    private final List<Double> powerDrawHistory = new ArrayList<>();
    private final List<Double> loopTimeHistory = new ArrayList<>();

    private long lastLoopStartTime = System.nanoTime();
    private double maxLoopTime = 0;
    private double avgLoopTime = 0;

    /**
     * Records the start of a periodic loop.
     * Call at the beginning of robotPeriodic().
     */
    public void recordLoopStart() {
        lastLoopStartTime = System.nanoTime();
    }

    /**
     * Records the end of a periodic loop and calculates loop time.
     * Call at the end of robotPeriodic().
     */
    public void recordLoopEnd() {
        long loopTimeNanos = System.nanoTime() - lastLoopStartTime;
        double loopTimeMs = loopTimeNanos / 1_000_000.0;

        loopTimeHistory.add(loopTimeMs);
        if (loopTimeHistory.size() > MAX_HISTORY_SIZE) {
            loopTimeHistory.remove(0);
        }

        // Update statistics
        maxLoopTime = loopTimeHistory.stream()
            .mapToDouble(Double::doubleValue)
            .max()
            .orElse(0);

        avgLoopTime = loopTimeHistory.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(0);

        // Post to SmartDashboard
        SmartDashboard.putNumber("Performance/Loop Time (ms)", loopTimeMs);
        SmartDashboard.putNumber("Performance/Max Loop Time (ms)", maxLoopTime);
        SmartDashboard.putNumber("Performance/Avg Loop Time (ms)", avgLoopTime);
    }

    /**
     * Records power consumption for trend analysis.
     */
    public void recordPowerConsumption(double powerWatts) {
        powerDrawHistory.add(powerWatts);
        if (powerDrawHistory.size() > MAX_HISTORY_SIZE) {
            powerDrawHistory.remove(0);
        }

        double avgPower = powerDrawHistory.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(0);

        double maxPower = powerDrawHistory.stream()
            .mapToDouble(Double::doubleValue)
            .max()
            .orElse(0);

        SmartDashboard.putNumber("Performance/Current Power (W)", powerWatts);
        SmartDashboard.putNumber("Performance/Avg Power (W)", avgPower);
        SmartDashboard.putNumber("Performance/Peak Power (W)", maxPower);
    }

    /**
     * Records command execution time for performance analysis.
     */
    public void recordCommandExecution(String commandName, long executionTimeMs) {
        commandExecutionTimes.add(executionTimeMs);
        if (commandExecutionTimes.size() > 100) {
            commandExecutionTimes.remove(0);
        }

        if (executionTimeMs > 50) { // Log slow commands
            System.out.println("Slow command: " + commandName + " took " + executionTimeMs + "ms");
        }
    }

    /**
     * Gets average loop execution time in milliseconds
     */
    public double getAverageLoopTime() {
        return avgLoopTime;
    }

    /**
     * Gets maximum loop execution time in milliseconds
     */
    public double getMaxLoopTime() {
        return maxLoopTime;
    }

    /**
     * Gets the count of loop iterations recorded
     */
    public int getLoopCount() {
        return loopTimeHistory.size();
    }

    /**
     * Resets all tracking data (call at start of match)
     */
    public void reset() {
        loopTimeHistory.clear();
        powerDrawHistory.clear();
        commandExecutionTimes.clear();
        maxLoopTime = 0;
        avgLoopTime = 0;
    }

    /**
     * Prints performance summary to console for debugging
     */
    public void printSummary() {
        System.out.println("===== Performance Summary =====");
        System.out.println("Average Loop Time: " + String.format("%.2f", avgLoopTime) + " ms");
        System.out.println("Max Loop Time: " + String.format("%.2f", maxLoopTime) + " ms");
        System.out.println("Total Loops: " + getLoopCount());
        System.out.println("Total Slow Commands: " + commandExecutionTimes.stream()
            .filter(t -> t > 50)
            .count());
    }
}

