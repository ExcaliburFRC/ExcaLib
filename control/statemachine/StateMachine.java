package frc.excalib.control.statemachine;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * A type-safe generic StateMachine runner utilizing Enums.
 * 
 * @param <E> The Enum type representing the states.
 */
public class StateMachine<E extends Enum<E>> {
    private final Map<E, State> states = new HashMap<>();
    private final Map<E, Map<E, BooleanSupplier>> transitions = new HashMap<>();
    private E currentState;

    /**
     * @param initialState The initial starting state of the machine.
     */
    public StateMachine(E initialState) {
        this.currentState = initialState;
    }

    /**
     * Registers a behavior for a specific state.
     * 
     * @param stateEnum The Enum representing this state.
     * @param behavior  The State behavior.
     */
    public void addState(E stateEnum, State behavior) {
        states.put(stateEnum, behavior);
    }

    /**
     * Registers a transition condition from one state to another.
     * 
     * @param from      The state to transition from.
     * @param to        The state to transition to.
     * @param condition The supplier that triggers the transition when true.
     */
    public void addTransition(E from, E to, BooleanSupplier condition) {
        transitions.computeIfAbsent(from, k -> new HashMap<>()).put(to, condition);
    }

    /**
     * @return The active state Enum.
     */
    public E getCurrentState() {
        return currentState;
    }

    /**
     * Processes the current state's execution and checks transitions.
     * Call this in the subsystem's periodic() loop.
     */
    public void update() {
        // Run active state execution
        State behavior = states.get(currentState);
        if (behavior != null) {
            behavior.execute();
        }

        // Evaluate transitions from current state
        Map<E, BooleanSupplier> activeTransitions = transitions.get(currentState);
        if (activeTransitions != null) {
            for (Map.Entry<E, BooleanSupplier> entry : activeTransitions.entrySet()) {
                if (entry.getValue().getAsBoolean()) {
                    transitionTo(entry.getKey());
                    break;
                }
            }
        }
    }

    /**
     * Force transitions to a specific state.
     * 
     * @param newState The state Enum to transition to.
     */
    public void transitionTo(E newState) {
        State oldState = states.get(currentState);
        if (oldState != null) {
            oldState.end();
        }

        currentState = newState;

        State nextState = states.get(currentState);
        if (nextState != null) {
            nextState.init();
        }
    }
}
