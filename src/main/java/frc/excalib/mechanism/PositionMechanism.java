package frc.excalib.mechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.encoder.Encoder;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.Motor;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

public class PositionMechanism extends Mechanism {
    private double m_setpoint;
    private PIDController m_pidController;
    private BiFunction<Double, Double, Double> m_feedforwardCalculator;
    private SoftLimit m_limit;

    public Trigger m_atSetpointTrigger;

    public PositionMechanism(Motor motor, Gains gains) {
        super(motor);
        m_pidController = gains.getPIDcontroller();

        m_atSetpointTrigger = new Trigger(()-> false);
        m_feedforwardCalculator = ((a, b)-> 0.0);
    }

    public PositionMechanism withTolerance(double tolerance) {
        m_atSetpointTrigger = new Trigger(() -> MathUtil.isNear(this.m_setpoint, super.logMechanismPosition(), tolerance));
        return this;
    }

    public PositionMechanism withFeedforwardCalculator(BiFunction<Double, Double, Double> calculator) {
        m_feedforwardCalculator = calculator;
        return this;
    }

    public PositionMechanism withLimit(SoftLimit limit) {
        m_limit = limit;
        return this;
    }

    public PositionMechanism withExternalEncoder(Encoder encoder) {
        super.addExternalEncoder(encoder);
        return this;
    }

    public Command setPositionCommand(DoubleSupplier positionSupplier, TrapezoidProfile profile, SubsystemBase... requirements) {
        return new RunCommand(() -> {
            State state = profile.calculate(
                    0.02,
                    new State(super.logMechanismPosition(), super.logMechanismVelocity()),
                    new State(positionSupplier.getAsDouble(), 0)
            );
            setPosition(state.position);
        }, requirements);
    }

    public void setPosition(double position) {
        m_setpoint = position;

        if (m_limit != null) {
            if (m_limit instanceof ContinuousSoftLimit) m_setpoint = ((ContinuousSoftLimit) m_limit).getSetpoint(super.logMechanismPosition(), position);
            else m_setpoint = m_limit.limit(position);
        }

        double pid = m_pidController.calculate(super.logMechanismPosition(), m_setpoint);
        double ff = m_feedforwardCalculator.apply(super.logMechanismPosition(), super.logMechanismVelocity());
        super.setVoltage(pid + ff);
    }
}
