// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.excalib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Utility class for creating and composing common robot commands.
 * Reduces boilerplate and ensures consistency across the codebase.
 */
public class CommandUtils {

    /**
     * Creates a command that logs a message when started.
     * Useful for debugging command execution order.
     */
    public static Command logCommand(String message) {
        return Commands.runOnce(() -> System.out.println(message));
    }

    /**
     * Creates a command that waits for a boolean supplier to become true.
     * Useful for synchronizing command sequences with subsystem states.
     */
    public static Command waitFor(java.util.function.BooleanSupplier condition) {
        return Commands.waitUntil(condition);
    }

    /**
     * Creates a command that runs a runnable and ensures specific subsystems are required.
     * Prevents concurrent execution on the same subsystem.
     */
    public static Command requireCommand(Runnable runnable, Subsystem... requirements) {
        Command cmd = Commands.runOnce(runnable);
        for (Subsystem requirement : requirements) {
            cmd.addRequirements(requirement);
        }
        return cmd;
    }

    /**
     * Creates a safety-checked command that only executes if a condition is met.
     * Returns an empty command if condition is false, preventing null pointer issues.
     */
    public static Command conditionalCommand(Command trueCommand, java.util.function.BooleanSupplier condition) {
        if (condition.getAsBoolean()) {
            return trueCommand;
        }
        return Commands.none();
    }

    /**
     * Creates a timeout-protected command that will be canceled after specified time.
     * Prevents infinite loops or stuck commands from blocking the robot.
     */
    public static Command withTimeout(Command command, double timeoutSeconds) {
        return command.withTimeout(timeoutSeconds);
    }

    /**
     * Creates a command sequence with logging for debugging.
     * Logs when starting and finishing each command.
     */
    public static Command debugSequence(String sequenceName, Command... commands) {
        return logCommand("Starting sequence: " + sequenceName)
                .andThen(Commands.sequence(commands))
                .andThen(logCommand("Finished sequence: " + sequenceName));
    }
}

