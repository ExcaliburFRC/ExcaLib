package frc.excalib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;
import java.util.function.Supplier;

/**
 * A command that maps a key of type T to a corresponding command.
 * This allows dynamic selection and execution of commands based on a supplied key.
 *
 * @param <T> The type of the key used to map to commands.
 */
public class MapCommand<T> extends Command {
    private final Map<T, Command> m_map; // A map that associates keys of type T with commands.
    private Command m_command; // The currently selected command to execute.
    private final Supplier<T> m_t; // A supplier that provides the key to select the command.

    /**
     * Constructs a new MapCommand.
     *
     * @param map A map associating keys of type T with commands.
     * @param t A supplier that provides the key to select the command.
     */
    public MapCommand(Map<T, Command> map, Supplier<T> t) {
        m_map = map;
        m_t = t;
        // Add the requirements of all commands in the map to this command.
        map.forEach((key, value) -> this.addRequirements(value.getRequirements()));
    }

    /**
     * Initializes the command by selecting the appropriate command from the map
     * based on the key provided by the supplier.
     * If no command is found for the key, a default "none" command is used.
     */
    @Override
    public void initialize() {
        m_command = m_map.get(m_t.get());
        if (m_command == null) {
            m_command = Commands.none(); // Use a default "none" command if no match is found.
        }
        m_command.initialize(); // Initialize the selected command.
    }

    /**
     * Executes the currently selected command.
     */
    @Override
    public void execute() {
        m_command.execute();
    }

    /**
     * Ends the currently selected command.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    /**
     * Checks if the currently selected command has finished execution.
     *
     * @return True if the selected command is finished, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return this.m_command.isFinished();
    }
}