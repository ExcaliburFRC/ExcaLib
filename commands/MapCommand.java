package frc.excalib.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.util.Map;
import java.util.function.Supplier;

public class MapCommand<T> extends Command {
    private final Map<T, Command> m_map;
    private Command m_command;
    private Supplier<T> m_t;

    public MapCommand(Map<T, Command> map, Supplier<T> t) {
        m_map = map;
        m_t = t;
        map.forEach((key, value) -> this.addRequirements(value.getRequirements()));
    }

    @Override
    public void initialize() {
        m_command = m_map.get(m_t.get());
        if (m_command == null) {
            m_command = Commands.none();
        }
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean inturupted) {
        m_command.end(inturupted);
    }

    @Override
    public boolean isFinished() {
        return this.m_command.isFinished();
    }
}
