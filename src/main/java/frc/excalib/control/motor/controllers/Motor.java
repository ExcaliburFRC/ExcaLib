package frc.excalib.control.motor.controllers;

import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

/**
 * Interface representing a motor controller with various control and configuration methods.
 */
public interface Motor {

    /**
     * Stops the motor by setting its output to zero.
     */
    void stopMotor();

    /**
     * Sets the motor output to a specific voltage.
     *
     * @param voltage The desired voltage to apply to the motor.
     */
    void setVoltage(double voltage);

    /**
     * Sets the motor output as a percentage of its maximum output.
     *
     * @param percentage The desired output percentage (range: -1.0 to 1.0).
     */
    void setPercentage(double percentage);

    /**
     * Configures the motor to follow another motor.
     *
     * @param mainMotorID The device ID of the motor to follow.
     */
    void setFollower(int mainMotorID);

    /**
     * Sets the idle state of the motor (e.g., brake or coast mode).
     *
     * @param idleMode The desired idle state.
     */
    void setIdleState(IdleState idleMode);

    /**
     * Retrieves the current idle state of the motor.
     *
     * @return The current idle state.
     */
    IdleState getIdleState();

    /**
     * Retrieves the device ID of the motor.
     *
     * @return The device ID.
     */
    int getDeviceID();

    /**
     * Retrieves the current position of the motor.
     *
     * @return The motor position in units defined by the position conversion factor.
     */
    double getMotorPosition();

    /**
     * Retrieves the current velocity of the motor.
     *
     * @return The motor velocity in units defined by the velocity conversion factor.
     */
    double getMotorVelocity();

    /**
     * Retrieves the current drawn by the motor.
     *
     * @return The current in amperes.
     */
    double getCurrent();

    /**
     * Retrieves the voltage being applied to the motor.
     *
     * @return The voltage in volts.
     */
    double getVoltage();

    /**
     * Retrieves the temperature of the motor.
     *
     * @return The temperature in degrees Celsius.
     */
    double getTemperature();

    /**
     * Sets a soft limit for the motor in a specific direction.
     *
     * @param directionState The direction (forward or reverse) for the soft limit.
     * @param limit          The position limit in units defined by the position conversion factor.
     */
    void setSoftLimit(DirectionState directionState, float limit);

    /**
     * Sets the motor's direction to be inverted or not.
     *
     * @param mode The desired direction state (inverted or not).
     */
    void setInverted(DirectionState mode);

    /**
     * Sets the conversion factor for motor position measurements.
     *
     * @param conversionFactor The factor to convert motor position to desired units.
     */
    void setPositionConversionFactor(double conversionFactor);

    /**
     * Sets the conversion factor for motor velocity measurements.
     *
     * @param conversionFactor The factor to convert motor velocity to desired units.
     */
    void setVelocityConversionFactor(double conversionFactor);

    /**
     * Sets the current limits for the motor.
     *
     * @param stallLimit The current limit during stall conditions in amperes.
     * @param freeLimit  The current limit during free-running conditions in amperes.
     */
    void setCurrentLimit(int stallLimit, int freeLimit);

    /**
     * Sets the motor's position to a specific value.
     *
     * @param position The desired position in units defined by the position conversion factor.
     */
    void setMotorPosition(double position);
}