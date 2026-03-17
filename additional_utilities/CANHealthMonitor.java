// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.additional_utilities;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Monitor CAN bus health to detect communication issues with motor controllers,
 * CANcoders, and other CAN devices before they cause runtime failures.
 */
public class CANHealthMonitor {
    private final Alert canBusAlert = new Alert("CAN Bus Error - Check connections", Alert.AlertType.kWarning);
    private int lastErrorCount = 0;

    /**
     * Updates CAN bus health status. Call in robotPeriodic().
     */
    public void update() {
        try {
            CANStatus canStatus = new CANStatus();
            // Get CAN bus statistics from HAL
            CANJNI.getCANStatus(canStatus);
            int errorCount = canStatus.busOffCount;

            // Check if error count has increased (indicates recent CAN errors)
            canBusAlert.set(errorCount > lastErrorCount);

            lastErrorCount = errorCount;

            // Post diagnostic data
            SmartDashboard.putNumber("CAN/Error Count", errorCount);
            SmartDashboard.putBoolean("CAN/Bus OK", errorCount == 0);
        } catch (Exception e) {
            System.err.println("Error reading CAN status: " + e.getMessage());
            canBusAlert.set(true);
        }
    }

    /**
     * Gets the current CAN error count
     */
    public int getErrorCount() {
        return lastErrorCount;
    }

    /**
     * Checks if CAN bus is healthy
     */
    public boolean isHealthy() {
        return lastErrorCount == 0;
    }
}

