/**
 * © 2025 Excalibur FRC. All rights reserved.
 * This file is part of ExcaLIb and may not be copied, modified,
 * or distributed without permission, except as permitted by license.
 * learn more at - https://github.com/ExaliburFRC/ExcaLIb
 */

package frc.excalib.mechanisms;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.control.motor.motor_specs.IdleState;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static monologue.Annotations.Log;

/**
 * A class representing a generic Mechanism
 */
public class Mechanism implements Logged {
    protected final Motor m_motor;
    protected final MutVoltage m_appliedVoltage = Volts.mutable(0);
    protected final MutAngle m_radians = Radians.mutable(0);
    protected final MutDistance m_meter = Meters.mutable(0);
    protected final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
    protected final MutLinearVelocity m_Linearvelocity = MetersPerSecond.mutable(0);
    private final IdleState m_DEFAULT_IDLE_STATE;

    /**
     * @param motor the motor controlling the mechanism
     */
    public Mechanism(Motor motor) {
        m_motor = motor;
        m_DEFAULT_IDLE_STATE = m_motor.getIdleState();
    }

    /**
     * @param output set the duty cycle output
     */
    public void setOutput(double output) {
        m_motor.setPercentage(output);
    }

    /**
     * @param voltage set the voltage cycle output
     */
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    /**
     * @param outputSupplier the dynamic setpoint for the mechanism (voltage)
     * @return a command which controls the mechanism manually
     */
    public Command manualCommand(DoubleSupplier outputSupplier, SubsystemBase... requirements) {
        return Commands.runEnd(
                () -> setOutput(outputSupplier.getAsDouble()),
                () -> setOutput(0),
                requirements
        );
    }

    /**
     * @return an instant command to stop the motor
     */
    public Command stopMotorCommand(SubsystemBase... requirements) {
        return new InstantCommand(() -> setOutput(0), requirements);
    }

    /**
     * @return the velocity
     */
    @Log.NT
    public double logVelocity() {
        return m_motor.getMotorVelocity();
    }

    /**
     * @return the position
     */
    @Log.NT
    public double logPosition() {
        return m_motor.getMotorPosition();
    }

    @Log.NT
    public double logVoltage() {
        return m_motor.getVoltage();
    }

    @Log.NT
    public double logCurrent() {
        return m_motor.getCurrent();
    }

    /**
     * Creates a SysIdRoutine for linear mechanisms.
     *
     * @param subsystem   the subsystem associated with the mechanism
     * @param sensorInput a supplier providing the sensor input for the mechanism
     * @param config      the configuration for the SysIdRoutine
     * @return a SysIdRoutine configured for linear mechanisms
     */
    private SysIdRoutine getLinearSysIdRoutine(SubsystemBase subsystem, DoubleSupplier sensorInput, SysidConfig config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> m_motor.setVoltage(volts.in(Volts)),
                        log -> log.motor("motor")
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_motor.getVoltage(), Volts))
                                .linearPosition(m_meter.mut_replace(sensorInput.getAsDouble(), Meters))
                                .linearVelocity(m_Linearvelocity.mut_replace(logVelocity(), MetersPerSecond)),
                        subsystem
                )
        );
    }

    /**
     * Creates a SysIdRoutine for angular mechanisms.
     *
     * @param subsystem   the subsystem associated with the mechanism
     * @param sensorInput a supplier providing the sensor input for the mechanism
     * @param config      the configuration for the SysIdRoutine
     * @return a SysIdRoutine configured for angular mechanisms
     */
    private SysIdRoutine getAngularSysIdRoutine(SubsystemBase subsystem, DoubleSupplier sensorInput, SysidConfig config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> m_motor.setVoltage(volts.in(Volts)),
                        log -> log.motor("motor")
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_motor.getVoltage(), Volts))
                                .angularPosition(m_radians.mut_replace(sensorInput.getAsDouble(), Radians))
                                .angularVelocity(m_velocity.mut_replace(logVelocity(), RadiansPerSecond)),
                        subsystem
                )
        );
    }

    /**
     * @return a command which puts the mechanism on coast mode
     */
    public Command coastCommand(SubsystemBase... requirements) {
        return new StartEndCommand(
                () -> m_motor.setIdleState(IdleState.COAST),
                () -> m_motor.setIdleState(m_DEFAULT_IDLE_STATE),
                requirements
        ).ignoringDisable(true);
    }

    /**
     * Creates a SysIdRoutine for performing a quasistatic test on the mechanism.
     *
     * @param direction the direction of the test (forward or backward)
     * @param subsystem the subsystem associated with the mechanism
     * @param positionSupplier a supplier providing the position of the mechanism
     * @param config the configuration for the SysIdRoutine
     * @param isLinear true if the mechanism is linear, false if it is angular
     * @return a Command that performs a quasistatic test
     */
    public Command sysIdQuasistatic(Direction direction,SubsystemBase subsystem, DoubleSupplier positionSupplier, SysidConfig config, boolean isLinear) {
        if (isLinear) return getLinearSysIdRoutine(subsystem, positionSupplier, config).quasistatic(direction);
        return getAngularSysIdRoutine(subsystem, positionSupplier, config).quasistatic(direction);
    }

    /**
     * Creates a SysIdRoutine for performing a dynamic test on the mechanism.
     *
     * @param direction the direction of the test (forward or backward)
     * @param subsystem the subsystem associated with the mechanism
     * @param positionSupplier a supplier providing the position of the mechanism
     * @param config the configuration for the SysIdRoutine
     * @param isLinear true if the mechanism is linear, false if it is angular
     * @return a Command that performs a dynamic test
     */
    public Command sysIdDynamic(Direction direction, SubsystemBase subsystem, DoubleSupplier positionSupplier, SysidConfig config, boolean isLinear) {
        if (isLinear)
            return getLinearSysIdRoutine(subsystem, positionSupplier, config).dynamic(direction);
        return getAngularSysIdRoutine(subsystem, positionSupplier, config).dynamic(direction).withName("quadForward");
    }
}
