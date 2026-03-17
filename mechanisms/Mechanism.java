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
import static monologue.Annotations.*;

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
        m_motor.setMotorVoltage(voltage);
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
        return  m_motor.getVoltage();
    }

    @Log.NT
    public double logCurrent() {
        return m_motor.getCurrent();
    }

    private SysIdRoutine getLinearSysIdRoutine(SubsystemBase subsystem, DoubleSupplier sensorInput, SysidConfig config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> m_motor.setMotorVoltage(volts.in(Volts)),
                        log -> log.motor("motor")
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_motor.getVoltage(), Volts))
                                .linearPosition(m_meter.mut_replace(sensorInput.getAsDouble(), Meters))
                                .linearVelocity(m_Linearvelocity.mut_replace(logVelocity(), MetersPerSecond)),
                        subsystem));
    }

    private SysIdRoutine getAngularSysIdRoutine(SubsystemBase subsystem, DoubleSupplier sensorInput, SysidConfig config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> m_motor.setMotorVoltage(volts.in(Volts)),
                        log -> log.motor("MOTOR")
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_motor.getVoltage(), Volts))
                                .angularPosition(m_radians.mut_replace(sensorInput.getAsDouble(), Radians))
                                .angularVelocity(m_velocity.mut_replace(logVelocity(), RadiansPerSecond)),
                        subsystem));
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, SubsystemBase subsystem, DoubleSupplier positionSupplier, SysidConfig config, boolean isLinear) {
        if (isLinear) return getLinearSysIdRoutine(subsystem, positionSupplier, config).quasistatic(direction);
        return getAngularSysIdRoutine(subsystem, positionSupplier, config).quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, SubsystemBase subsystem, DoubleSupplier positionSupplier, SysidConfig config, boolean isLinear) {
        if (isLinear)
            return getLinearSysIdRoutine(subsystem, positionSupplier, config).dynamic(direction);
        return getAngularSysIdRoutine(subsystem, positionSupplier, config).dynamic(direction).withName("quadForward");
    }
}
