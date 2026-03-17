// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tracks and monitors controller state for diagnostics and debugging.
 * Helps identify controller connection issues and input conflicts.
 */
public class ControllerStateTracker {
    private final GenericHID controller;
    private final String controllerName;
    private boolean wasConnected = false;
    private int disconnectCount = 0;
    private long lastConnectedTime = System.currentTimeMillis();

    public ControllerStateTracker(GenericHID controller, String controllerName) {
        this.controller = controller;
        this.controllerName = controllerName;
    }

    /**
     * Updates controller connection state. Call in robotPeriodic().
     */
    public void update() {
        boolean isConnected = controller.isConnected();

        // Detect connection state changes
        if (isConnected && !wasConnected) {
            // Controller just connected
            System.out.println(controllerName + " connected");
            lastConnectedTime = System.currentTimeMillis();
        } else if (!isConnected && wasConnected) {
            // Controller just disconnected
            System.err.println(controllerName + " DISCONNECTED");
            disconnectCount++;
        }

        wasConnected = isConnected;

        // Post diagnostics
        SmartDashboard.putBoolean("Controller/" + controllerName + "/Connected", isConnected);
        SmartDashboard.putNumber("Controller/" + controllerName + "/Disconnect Count", disconnectCount);
        SmartDashboard.putNumber("Controller/" + controllerName + "/POV", controller.getPOV());
    }

    /**
     * Gets the number of disconnects that have occurred
     */
    public int getDisconnectCount() {
        return disconnectCount;
    }

    /**
     * Checks if controller is currently connected
     */
    public boolean isConnected() {
        return wasConnected;
    }

    /**
     * Gets time since last connection (in seconds)
     */
    public long getSecondsSinceConnection() {
        return (System.currentTimeMillis() - lastConnectedTime) / 1000;
    }

    /**
     * Reset disconnect counter
     */
    public void resetDisconnectCount() {
        disconnectCount = 0;
    }
}

