package frc.excalib.commands;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

public class ContinuouslyConditionalCommand extends Command {
    // TODO: check & debug
    private final Command m_onTrue;
    private final Command m_onFalse;
    private final BooleanSupplier m_condition;
    private final Command m_continuouslyConditionalCommand;

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

    private Command getCurrentCommand(){
        if (m_condition.getAsBoolean()) return m_onTrue;
        return m_onFalse;
    }

    private Command getOtherCommand(){
        if (m_condition.getAsBoolean()) return m_onFalse;
        return m_onTrue;
    }

    public void initialize() {
        m_continuouslyConditionalCommand.schedule();
    }
}
