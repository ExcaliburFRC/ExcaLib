package frc.excalib.mechanisms.linear_extension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;

public class LinearExtension extends Mechanism {
    private final DoubleSupplier m_positionSupplier;
    private final DoubleSupplier m_angleSupplier;
    private final PIDController m_PIDController;
    private final double m_tolerance;
    private final Gains m_gains;

    private final TrapezoidProfile.Constraints m_constraints;

    public LinearExtension(Motor motor, DoubleSupplier positionSupplier, DoubleSupplier angleSupplier, Gains gains, TrapezoidProfile.Constraints constraints, double tolerance) {
        super(motor);
        m_positionSupplier = positionSupplier;
        m_angleSupplier = angleSupplier;
        m_gains = gains;
        m_PIDController = new PIDController(gains.kp, gains.ki, gains.kd);
        m_constraints = constraints;
        m_tolerance = tolerance;
    }

    public Command extendCommand(DoubleSupplier lengthSetPoint, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
            TrapezoidProfile.State state =
                    profile.calculate(
                            0.02,
                            new TrapezoidProfile.State(
                                    m_positionSupplier.getAsDouble(),
                                    super.m_motor.getMotorVelocity()),
                            new TrapezoidProfile.State(lengthSetPoint.getAsDouble(), 0)
                    );
            double pidValue = m_PIDController.calculate(m_positionSupplier.getAsDouble(), state.position);
            double ff =
                    (Math.abs(m_positionSupplier.getAsDouble() - lengthSetPoint.getAsDouble()) > m_tolerance) ?

                                    m_gains.ks * Math.signum(state.velocity) +
                                    m_gains.kv * state.velocity +
                                    m_gains.kg * Math.sin(m_angleSupplier.getAsDouble()) :

                                    m_gains.kg * Math.sin(m_angleSupplier.getAsDouble());
            double output = ff + pidValue;
            setVoltage(output);
        }, requirements);
    }

    public double logVoltage() {
        return m_motor.getVoltage();
    }

    public double logVelocity() {
        return m_motor.getMotorVelocity();
    }

    public double logPosition() {
        return m_positionSupplier.getAsDouble();
    }

}