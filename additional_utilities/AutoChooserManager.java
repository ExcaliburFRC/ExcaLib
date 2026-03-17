// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.additional_utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Centralized manager for autonomous command selection.
 * Handles initialization and validation of auto selections to prevent runtime crashes.
 */
public class AutoChooserManager {
    private static final String NULL_AUTO = "/ null Auto";
    private static final String SAFE_AUTO = "Safe Auto"; // Fallback option

    private final SendableChooser<String> autoChooser;

    public AutoChooserManager() {
        this.autoChooser = new SendableChooser<>();
        initializeChooser();
    }

    /**
     * Initializes the auto chooser with all available autos from PathPlanner.
     * Provides a safe default option if no auto is selected.
     */
    private void initializeChooser() {
        // Set null auto as default to indicate no selection
        autoChooser.setDefaultOption(NULL_AUTO, NULL_AUTO);

        // Add all available auto names from PathPlanner
        try {
            for (String autoName : AutoBuilder.getAllAutoNames()) {
                autoChooser.addOption(autoName, autoName);
            }
        } catch (Exception e) {
            System.err.println("Error loading autonomous paths: " + e.getMessage());
            // Gracefully handle path loading errors
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Gets the selected auto name, validating that it exists.
     * Returns null if no valid auto is selected.
     */
    public String getSelectedAuto() {
        String selected = autoChooser.getSelected();

        // Return null if null auto is selected or selection is null
        if (selected == null || selected.equals(NULL_AUTO)) {
            return null;
        }

        // Validate that the selected auto actually exists
        if (AutoBuilder.getAllAutoNames().contains(selected)) {
            return selected;
        }

        // Return null if selection is invalid (path doesn't exist)
        System.err.println("Selected auto '" + selected + "' not found in available autos");
        return null;
    }

    /**
     * Checks if an auto selection has been made.
     */
    public boolean isAutoSelected() {
        return getSelectedAuto() != null;
    }

    /**
     * Gets the SendableChooser for dashboard integration.
     */
    public SendableChooser<String> getChooser() {
        return autoChooser;
    }
}

