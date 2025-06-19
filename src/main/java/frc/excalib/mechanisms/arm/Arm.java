package frc.excalib.mechanisms.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Represents a robotic arm mechanism with PID control and physical constraints.
 * Provides commands for moving the arm to a specified angle or following a dynamic setpoint.
 */
public class Arm extends Mechanism {
    /**
     * The mass properties of the arm.
     */
    private final Mass m_mass;
    /**
     * The PID controller used for angle control.
     */
    private final PIDController m_PIDController;
    /**
     * Supplies the current angle of the arm (in radians).
     */
    public final DoubleSupplier ANGLE_SUPPLIER;
    /**
     * Velocity feedforward gain.
     */
    public final double m_kv;
    /**
     * Static feedforward gain.
     */
    public final double m_ks;
    /**
     * Gravity feedforward gain.
     */
    public final double m_kg;
    /**
     * Soft limit for velocity constraints.
     */
    public final SoftLimit m_VELOCITY_LIMIT;

    /**
     * Constructs an Arm mechanism.
     *
     * @param motor         the motor controller for the arm
     * @param angleSupplier supplies the current arm angle (radians)
     * @param velocityLimit soft limit for velocity
     * @param gains         PID and feedforward gains
     * @param mass          mass properties of the arm
     */
    public Arm(Motor motor, DoubleSupplier angleSupplier,
               SoftLimit velocityLimit, Gains gains, Mass mass) {
        super(motor);
        ANGLE_SUPPLIER = angleSupplier;
        m_VELOCITY_LIMIT = velocityLimit;
        m_kg = gains.kg;
        m_kv = gains.kv;
        m_ks = gains.ks;
        m_PIDController = new PIDController(gains.kp, gains.ki, gains.kd);
        m_mass = mass;
    }

    /**
     * Creates a command to move the arm to a dynamic angle setpoint using PID and feedforward control.
     *
     * @param setpointSupplier  supplies the target angle setpoint (radians)
     * @param toleranceConsumer consumer that receives whether the arm is within the specified tolerance
     * @param maxOffSet         maximum allowed error for tolerance (radians)
     * @param requirements      subsystems required by this command
     * @return a command that moves the arm to the specified dynamic setpoint
     */
    public Command anglePositionControlCommand(
            DoubleSupplier setpointSupplier, Consumer<Boolean> toleranceConsumer,
            double maxOffSet, SubsystemBase... requirements) {
        final double dutyCycle = 0.02;
        return new RunCommand(
                () -> {
                    double error = setpointSupplier.getAsDouble() - ANGLE_SUPPLIER.getAsDouble();
                    double velocitySetpoint = error / dutyCycle;
                    velocitySetpoint = m_VELOCITY_LIMIT.limit(velocitySetpoint);
                    double phyOutput =
                            m_ks * Math.signum(velocitySetpoint) +
                                    m_kg * m_mass.getCenterOfMass().getX();
                    double pid = m_PIDController.calculate(
                            ANGLE_SUPPLIER.getAsDouble(),
                            setpointSupplier.getAsDouble()
                    );
                    double output = phyOutput + pid;
                    super.setVoltage(m_VELOCITY_LIMIT.limit(output));
                    toleranceConsumer.accept(Math.abs(error) < maxOffSet);
                }, requirements);
    }

    /**
     * Creates a command to move the arm to a fixed angle setpoint using PID and feedforward control.
     *
     * @param angle             the target angle setpoint (radians)
     * @param toleranceConsumer consumer that receives whether the arm is within the specified tolerance
     * @param maxOffSet         maximum allowed error for tolerance (radians)
     * @param requirements      subsystems required by this command
     * @return a command that moves the arm to the specified setpoint
     */
    public Command goToAngleCommand(double angle, Consumer<Boolean> toleranceConsumer, double maxOffSet, SubsystemBase... requirements) {
        return anglePositionControlCommand(() -> angle, toleranceConsumer, maxOffSet, requirements);
    }
}