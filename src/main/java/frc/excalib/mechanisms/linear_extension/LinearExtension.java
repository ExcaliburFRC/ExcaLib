package frc.excalib.mechanisms.linear_extension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * Represents a linear extension mechanism with PID and feedforward control.
 * Supports motion profiling using a trapezoidal profile and provides commands
 * for extending to a specified length with physical constraints.
 */
public class LinearExtension extends Mechanism {
    /** Supplies the current position of the extension (meters). */
    private final DoubleSupplier m_positionSupplier;
    /** Supplies the current angle of the extension (radians). */
    private final DoubleSupplier m_angleSupplier;
    /** PID controller for position control. */
    private final PIDController m_PIDController;
    /** Allowed position tolerance (meters). */
    private final double m_tolerance;
    /** Gains for PID and feedforward control. */
    private final Gains m_gains;
    /** Soft limit for the elevator hieght in meters */
    private final SoftLimit m_heightLimit;
    /** Motion profile constraints (max velocity and acceleration). */
    private final TrapezoidProfile.Constraints m_constraints;

    /**
     * Constructs a LinearExtension mechanism.
     *
     * @param motor           the motor controller for the extension
     * @param positionSupplier supplies the current extension position (meters)
     * @param angleSupplier    supplies the current extension angle (radians)
     * @param gains            PID and feedforward gains
     * @param constraints      trapezoidal motion profile constraints
     * @param tolerance        allowed position tolerance (meters)
     */
    public LinearExtension(Motor motor, DoubleSupplier positionSupplier, DoubleSupplier angleSupplier,
                           TrapezoidProfile.Constraints constraints, SoftLimit heightLimit,
                           Gains gains, double tolerance) {
        super(motor);
        m_positionSupplier = positionSupplier;
        m_angleSupplier = angleSupplier;
        m_gains = gains;
        m_PIDController = new PIDController(gains.kp, gains.ki, gains.kd);
        m_constraints = constraints;
        m_heightLimit = heightLimit;
        m_tolerance = tolerance;
    }

    /**
     * Creates a command to extend the mechanism to a dynamic length setpoint using
     * trapezoidal motion profiling, PID, and feedforward control.
     *
     * @param lengthSetpoint supplies the target extension length (meters)
     * @param requirements   subsystems required by this command
     * @return a command that extends the mechanism to the specified setpoint
     */
    public Command extendCommand(DoubleSupplier lengthSetpoint, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            double lengthLimitedSetpoint = m_heightLimit.limit(lengthSetpoint.getAsDouble());
            TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
            TrapezoidProfile.State state =
                    profile.calculate(
                            0.02,
                            new TrapezoidProfile.State(
                                    m_positionSupplier.getAsDouble(),
                                    super.m_motor.getMotorVelocity()),
                            new TrapezoidProfile.State(lengthLimitedSetpoint, 0)
                    );
            double pidValue = m_PIDController.calculate(m_positionSupplier.getAsDouble(), state.position);
            double ff =
                    (Math.abs(m_positionSupplier.getAsDouble() - lengthLimitedSetpoint) > m_tolerance) ?

                                    m_gains.ks * Math.signum(state.velocity) +
                                    m_gains.kv * state.velocity +
                                    m_gains.kg * Math.sin(m_angleSupplier.getAsDouble()) :

                                    m_gains.kg * Math.sin(m_angleSupplier.getAsDouble());
            double output = ff + pidValue;
            setVoltage(output);
        }, requirements);
    }

    /**
     * Logs the current voltage applied to the motor.
     *
     * @return the motor voltage (volts)
     */
    public double logVoltage() {
        return m_motor.getVoltage();
    }

    /**
     * Logs the current velocity of the motor.
     *
     * @return the motor velocity (meters per second)
     */
    public double logVelocity() {
        return m_motor.getMotorVelocity();
    }

    /**
     * Logs the current position of the extension.
     *
     * @return the extension position (meters)
     */
    public double logPosition() {
        return m_positionSupplier.getAsDouble();
    }

}