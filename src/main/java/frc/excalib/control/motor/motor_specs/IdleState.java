package frc.excalib.control.motor.motor_specs;

/**
 * Represents the idle state of a motor.
 * This enum is used to specify whether the motor is in a coasting state
 * or a braking state when idle.
 */
public enum IdleState {
    /**
     * Indicates that the motor is in a coasting state, allowing free movement.
     */
    COAST,

    /**
     * Indicates that the motor is in a braking state, resisting movement.
     */
    BRAKE
}