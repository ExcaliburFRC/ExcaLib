package frc.excalib.commands;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import java.util.function.BooleanSupplier;

/**
 * A command that continuously evaluates a condition and schedules one of two commands
 * based on the result. The evaluation and scheduling repeat until either command finishes.
 *
 * <p>This is useful for scenarios where the condition may change during execution,
 * and the command to run should switch accordingly, but the overall command ends
 * when either of the sub-commands finishes.</p>
 */
public class ContinuouslyConditionalCommand extends Command {
    /** Command to run when the condition is true. */
    private final Command m_onTrue;
    /** Command to run when the condition is false. */
    private final Command m_onFalse;
    /** The condition to evaluate. */
    private final BooleanSupplier m_condition;
    /** The internal command that handles continuous conditional execution. */
    private final Command m_continuouslyConditionalCommand;

    /**
     * Constructs a new Continuously Conditional Command.
     *
     * @param onTrue    the command to run when the condition is true
     * @param onFalse   the command to run when the condition is false
     * @param condition the condition to evaluate
     * @throws NullPointerException if any argument is null
     */
    public ContinuouslyConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.m_onTrue = ErrorMessages.requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
        this.m_onFalse = ErrorMessages.requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
        this.m_condition = ErrorMessages.requireNonNullParam(condition, "condition", "ConditionalCommand");

        m_continuouslyConditionalCommand = new ConditionalCommand(
                m_onTrue,
                m_onFalse,
                m_condition
        ).repeatedly().until(() -> m_onTrue.isFinished() || m_onFalse.isFinished());
    }

    /**
     * Returns the currently selected command based on the condition.
     *
     * @return the command corresponding to the current condition value
     */
    private Command getCurrentCommand(){
        if (m_condition.getAsBoolean()) return m_onTrue;
        return m_onFalse;
    }

    /**
     * Returns the command not currently selected by the condition.
     *
     * @return the command not corresponding to the current condition value
     */
    private Command getOtherCommand(){
        if (m_condition.getAsBoolean()) return m_onFalse;
        return m_onTrue;
    }

    /**
     * Initializes the command by scheduling the internal continuously conditional command.
     */
    public void initialize() {
        m_continuouslyConditionalCommand.schedule();
    }
}