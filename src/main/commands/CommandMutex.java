package frc.excalib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandMutex {
    private Command currentCommand;

    /**
     * Schedule a new command. Cancels the currently running one (if any).
     */
    public void schedule(Command newCommand) {
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }

        currentCommand = newCommand;
        if (newCommand != null) {
            CommandScheduler.getInstance().schedule(newCommand);
        }
    }

    /**
     * Cancel the currently running command (if any).
     */
    public void cancel() {
        if (currentCommand != null) {
            currentCommand.cancel();
            currentCommand = null;
        }
    }

    /**
     * Check if a command is running in this mutex.
     */
    public boolean isRunning() {
        return currentCommand != null && currentCommand.isScheduled();
    }

    /**
     * Get the currently running command.
     */
    public Command getCurrentCommand() {
        return currentCommand;
    }

    public Command scheduleCommand(Command newCommand) {
        return new InstantCommand(() -> schedule(newCommand));
    }

    public Command cancelCommand(){
        return new InstantCommand(this::cancel);
    }
}
