package frc.excalib.control.statemachine;

/**
 * A standard state lifecycle interface for the Excalib state machine.
 */
public interface State {
    /**
     * Called once when the state machine transitions into this state.
     */
    default void init() {}

    /**
     * Called periodically in the subsystem loop while this state is active.
     */
    default void execute() {}

    /**
     * Called once when transitioning out of this state.
     */
    default void end() {}

    /**
     * @return true if the state's operations are complete.
     */
    default boolean isDone() {
        return false;
    }
}
