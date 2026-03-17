// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for monitoring robot performance metrics and system health.
 * Provides real-time diagnostics for power consumption, battery voltage, and subsystem states.
 */
public class RobotDiagnostics {
    private final PowerDistribution pdh;

    // Performance metrics
    private double totalEnergy = 0.0; // Watt-hours
    private double peakCurrent = 0.0;
    private double peakVoltage = 0.0;
    private double minVoltage = 24.0;

    private long lastUpdateTime = 0;

    public RobotDiagnostics(PowerDistribution pdh) {
        this.pdh = pdh;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    /**
     * Updates performance metrics. Call this in robotPeriodic() for accurate tracking.
     */
    public void update() {
        double currentVoltage = pdh.getVoltage();
        double totalCurrent = pdh.getTotalCurrent();

        // Track peak and minimum voltages
        peakVoltage = Math.max(peakVoltage, currentVoltage);
        minVoltage = Math.min(minVoltage, currentVoltage);

        // Track peak current
        peakCurrent = Math.max(peakCurrent, totalCurrent);

        // Calculate energy consumption (power integration over time)
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 3600000.0; // Convert ms to hours
        double power = currentVoltage * totalCurrent; // Watts
        totalEnergy += power * dt;
        lastUpdateTime = currentTime;

        // Post to SmartDashboard for monitoring
        updateDashboard();
    }

    /**
     * Posts diagnostics data to SmartDashboard.
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("PDH/Voltage", pdh.getVoltage());
        SmartDashboard.putNumber("PDH/Total Current", pdh.getTotalCurrent());
        SmartDashboard.putNumber("PDH/Peak Current", peakCurrent);
        SmartDashboard.putNumber("PDH/Peak Voltage", peakVoltage);
        SmartDashboard.putNumber("PDH/Min Voltage", minVoltage);
        SmartDashboard.putNumber("PDH/Total Energy (Wh)", totalEnergy);
    }

    /**
     * Gets current battery voltage
     */
    public double getVoltage() {
        return pdh.getVoltage();
    }

    /**
     * Gets total current draw
     */
    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    /**
     * Gets current power consumption (Watts)
     */
    public double getPowerConsumption() {
        return pdh.getVoltage() * pdh.getTotalCurrent();
    }

    /**
     * Gets the peak current recorded during the match
     */
    public double getPeakCurrent() {
        return peakCurrent;
    }

    /**
     * Gets total energy consumed (Watt-hours)
     */
    public double getTotalEnergy() {
        return totalEnergy;
    }

    /**
     * Checks if the battery voltage is critically low
     */
    public boolean isBatteryLow(double threshold) {
        return getVoltage() < threshold;
    }

    /**
     * Reset tracking metrics (call at the start of each match)
     */
    public void reset() {
        totalEnergy = 0.0;
        peakCurrent = 0.0;
        peakVoltage = 0.0;
        minVoltage = 24.0;
        lastUpdateTime = System.currentTimeMillis();
    }
}

